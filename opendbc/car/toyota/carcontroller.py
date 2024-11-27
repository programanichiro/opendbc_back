import os
import math
from opendbc.car import Bus, carlog, apply_meas_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance, \
                        make_tester_present_msg, rate_limit, structs, ACCELERATION_DUE_TO_GRAVITY, DT_CTRL
from collections import deque
from opendbc.car.can_definitions import CanData
from opendbc.car.common.pid import PIDController
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.common.numpy_fast import clip, interp
from opendbc.car.common.pid import PIDController
from opendbc.car.secoc import add_mac, build_sync_mac
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.toyota import toyotacan
from opendbc.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        CarControllerParams, ToyotaFlags, \
                                        UNSUPPORTED_DSU_CAR
from opendbc.can.packer import CANPacker

Ecu = structs.CarParams.Ecu
LongCtrlState = structs.CarControl.Actuators.LongControlState
SteerControlType = structs.CarParams.SteerControlType
VisualAlert = structs.CarControl.HUDControl.VisualAlert

# The up limit allows the brakes/gas to unwind quickly leaving a stop,
# the down limit roughly matches the rate of ACCEL_NET, reducing PCM compensation windup
ACCEL_WINDUP_LIMIT = 3.0 * DT_CTRL * 3  # m/s^2 / frame
ACCEL_WINDDOWN_LIMIT = -5.0 * DT_CTRL * 3  # m/s^2 / frame

# LKA limits
# EPS faults if you apply torque while the steering rate is above 100 deg/s for too long
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 18  # tx control frames needed before torque can be cut
# MAX_STEER_RATE = 105  # これを現車で耐えられる可能な限り上げる
# MAX_STEER_RATE_FRAMES = 25 # こちらも耐えられる可能な限り上げる？

# EPS allows user torque above threshold for 50 frames before permanently faulting
MAX_USER_TORQUE = 500

# LTA limits
# EPS ignores commands above this angle and causes PCS to fault
MAX_LTA_ANGLE = 94.9461  # deg
MAX_LTA_DRIVER_TORQUE_ALLOWANCE = 150  # slightly above steering pressed allows some resistance when changing lanes

# PCM compensatory force calculation threshold interpolation values
COMPENSATORY_CALCULATION_THRESHOLD_V = [-0.2, -0.2, -0.05]  # m/s^2
COMPENSATORY_CALCULATION_THRESHOLD_BP = [0., 20., 32.]  # m/s

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.last_steer = 0
    self.last_angle = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.permit_braking = True
    self.steer_rate_counter = 0
    self.prohibit_neg_calculation = True
    self.distance_button = 0

    self.deque = deque([0] * 300, maxlen=300)

    self.pid = PIDController(0.5, 0.25, k_f=0.0, k_d=0.125,
                             pos_limit=self.params.ACCEL_MAX, neg_limit=self.params.ACCEL_MIN,
                             rate=1 / DT_CTRL / 3)

    self.aego = FirstOrderFilter(0.0, 0.25, DT_CTRL)

    self.error_rate = FirstOrderFilter(0.0, 0.25, DT_CTRL * 3)
    self.prev_error = 0.0

    self.pitch = FirstOrderFilter(0, 0.5, DT_CTRL)
    self.net_acceleration_request = FirstOrderFilter(0, 0.15, DT_CTRL * 3)
    self.pcm_accel_cmd = FirstOrderFilter(0, 0.15 + 0.2, DT_CTRL * 3)

    self.pcm_accel_compensation = FirstOrderFilter(0, 0.5, DT_CTRL * 3)

    # the PCM's reported acceleration request can sometimes mismatch aEgo, close the loop
    self.pcm_accel_net_offset = FirstOrderFilter(0, 1.0, DT_CTRL * 3)

    # aEgo also often lags behind the PCM request due to physical brake lag which varies by car,
    # so we error correct on the filtered PCM acceleration request using the actuator delay.
    # TODO: move the delay into the interface
    self.pcm_accel_net = FirstOrderFilter(0, self.CP.longitudinalActuatorDelay, DT_CTRL * 3)
    if not any(fw.ecu == Ecu.hybrid for fw in self.CP.carFw):
      self.pcm_accel_net.update_alpha(self.CP.longitudinalActuatorDelay + 0.2)

    self.last_brake_frame = 0

    self.packer = CANPacker(dbc_names[Bus.pt])
    self.accel = 0
    self.prev_accel = 0

    self.debug = 0
    self.debug2 = 0
    self.debug3 = 0

    self.secoc_lka_message_counter = 0
    self.secoc_lta_message_counter = 0
    self.secoc_prev_reset_counter = 0
    self.secoc_mismatch_counter = 0

    self.now_gear = structs.CarState.GearShifter.park
    self.lock_flag = False
    self.lock_speed = 0
    try:
      with open('../../../run_auto_lock.txt','r') as fp:
        lock_speed_str = fp.read() #ロックするスピードをテキストで30みたいに書いておく。ファイルが無いか0でオートロック無し。
        if lock_speed_str:
          self.lock_speed = int(lock_speed_str)
    except Exception as e:
      pass
    self.before_ang = 0
    self.before_ang_ct = 0
    self.new_steers = []

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    stopping = actuators.longControlState == LongCtrlState.stopping
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    if len(CC.orientationNED) == 3:
      self.pitch.update(CC.orientationNED[1])

    # *** control msgs ***
    can_sends = []

    # *** handle secoc reset counter increase ***
    if self.CP.flags & ToyotaFlags.SECOC.value:
      if CS.secoc_synchronization['RESET_CNT'] != self.secoc_prev_reset_counter:
        self.secoc_lka_message_counter = 0
        self.secoc_lta_message_counter = 0
        self.secoc_prev_reset_counter = CS.secoc_synchronization['RESET_CNT']

        expected_mac = build_sync_mac(self.secoc_key, int(CS.secoc_synchronization['TRIP_CNT']), int(CS.secoc_synchronization['RESET_CNT']))
        if int(CS.secoc_synchronization['AUTHENTICATOR']) != expected_mac and self.secoc_mismatch_counter < 100:
          carlog.error("SecOC synchronization MAC mismatch, wrong key?")
          self.secoc_mismatch_counter += 1

    # *** steer torque ***
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    if (self.CP.carFingerprint not in TSS2_CAR) and (CS.knight_scanner_bit3 & 0x02): # knight_scanner_bit3.txt ⚪︎⚫︎⚪︎
      if abs(self.before_ang - CS.out.steeringAngleDeg) > 3.0/100: #1秒で3度以上
        # ハンドルが大きく動いたら
        self.before_ang_ct *= 0.9
      else:
        if self.before_ang_ct < 100:
          self.before_ang_ct += 1
      self.before_ang = CS.out.steeringAngleDeg

      new_steer0 = new_steer
      self.new_steers.append(float(new_steer0))
      if len(self.new_steers) > 10:
        self.new_steers.pop(0)
        #5〜ct〜55 -> 1〜10回の平均
        l = int(self.before_ang_ct) / 5
        l = int(1 if l < 1 else (l if l < 10 else 10))
        sum_steer = 0
        for i in range(l): #i=0..9
          sum_steer += self.new_steers[9-i]
        new_steer = sum_steer / l
        # with open('/tmp/debug_out_v','w') as fp:
        #   fp.write("ct:%d,%+.2f/%+.2f(%+.3f)" % (int(l),new_steer,new_steer0,new_steer-new_steer0))
    try:
      with open('/tmp/lane_d_info.txt','r') as fp:
        lane_d_info_str = fp.read()
        if lane_d_info_str:
          lane_d_info = float(lane_d_info_str)
          # nn_lane_d_info = 0 if new_steer == 0 else lane_d_info / new_steer
          # with open('/tmp/debug_out_v','w') as fp:
          #   fp.write('ns:%.7f/%.5f(%.1f%%)' % (new_steer,lane_d_info,nn_lane_d_info*100))
          #new_steerはマイナスで右に曲がる。
          new_steer -= lane_d_info * 100 * 30 #引くとセンターへ車体を戻す。
    except Exception as e:
      pass
    apply_steer = apply_meas_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params)

    # >100 degree/sec steering fault prevention
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, lat_active,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    if not lat_active:
      apply_steer = 0

    # *** steer angle ***
    if self.CP.steerControlType == SteerControlType.angle:
      # If using LTA control, disable LKA and set steering angle command
      apply_steer = 0
      apply_steer_req = False
      if self.frame % 2 == 0:
        # EPS uses the torque sensor angle to control with, offset to compensate
        apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        # Angular rate limit based on speed
        apply_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgoRaw, self.params)

        if not lat_active:
          apply_angle = CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        self.last_angle = clip(apply_angle, -MAX_LTA_ANGLE, MAX_LTA_ANGLE)

    self.last_steer = apply_steer

    # toyota can trace shows STEERING_LKA at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    steer_command = toyotacan.create_steer_command(self.packer, apply_steer, apply_steer_req)
    if self.CP.flags & ToyotaFlags.SECOC.value:
      # TODO: check if this slow and needs to be done by the CANPacker
      steer_command = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_lka_message_counter,
                              steer_command)
      self.secoc_lka_message_counter += 1
    can_sends.append(steer_command)

    # STEERING_LTA does not seem to allow more rate by sending faster, and may wind up easier
    if self.frame % 2 == 0 and self.CP.carFingerprint in TSS2_CAR:
      lta_active = lat_active and self.CP.steerControlType == SteerControlType.angle
      # cut steering torque with TORQUE_WIND_DOWN when either EPS torque or driver torque is above
      # the threshold, to limit max lateral acceleration and for driver torque blending respectively.
      full_torque_condition = (abs(CS.out.steeringTorqueEps) < self.params.STEER_MAX and
                               abs(CS.out.steeringTorque) < MAX_LTA_DRIVER_TORQUE_ALLOWANCE)

      # TORQUE_WIND_DOWN at 0 ramps down torque at roughly the max down rate of 1500 units/sec
      torque_wind_down = 100 if lta_active and full_torque_condition else 0
      can_sends.append(toyotacan.create_lta_steer_command(self.packer, self.CP.steerControlType, self.last_angle,
                                                          lta_active, self.frame // 2, torque_wind_down))

      if self.CP.flags & ToyotaFlags.SECOC.value:
        lta_steer_2 = toyotacan.create_lta_steer_command_2(self.packer, self.frame // 2)
        lta_steer_2 = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_lta_message_counter,
                              lta_steer_2)
        self.secoc_lta_message_counter += 1
        can_sends.append(lta_steer_2)

    # *** gas and brake ***

    prev_aego = self.aego.x
    self.aego.update(CS.out.aEgoBlended)
    jEgo = (self.aego.x - prev_aego) / DT_CTRL
    # TODO: adjust for hybrid
    future_aego = CS.out.aEgoBlended + jEgo * 0.5

    self.debug = jEgo
    self.debug2 = future_aego
    self.debug3 = self.aego.x

    # on entering standstill, send standstill request
    if CS.out.standstill and not self.CP.openpilotLongitudinalControl and not self.last_standstill and (self.CP.carFingerprint not in NO_STOP_TIMER_CAR):
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      # pcm entered standstill or it's disabled
      self.standstill_req = False

    self.last_standstill = CS.out.standstill

    # handle UI messages
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    lead = hud_control.leadVisible or CS.out.vEgo < 12.  # at low speed we always assume the lead is present so ACC can be engaged

    if self.lock_speed > 0: #auto door lock , unlock
      gear = CS.out.gearShifter
      if self.now_gear != gear or (CS.out.doorOpen and self.lock_flag == True): #ギアが変わるか、ドアが開くか。
        if gear == structs.CarState.GearShifter.park and CS.out.doorOpen == False: #ロックしたまま開ける時の感触がいまいちなので、パーキングでアンロックする。
          can_sends.append(CanData(0x750, b'\x40\x05\x30\x11\x00\x40\x00\x00', 0)) #auto unlock
        self.lock_flag = False #ドアが空いてもフラグはおろす。
      elif gear == structs.CarState.GearShifter.drive and self.lock_flag == False and CS.out.vEgo >= self.lock_speed/3.6: #時速30km/h以上でオートロック
        can_sends.append(CanData(0x750, b'\x40\x05\x30\x11\x00\x80\x00\x00', 0)) #auto lock
        self.lock_flag = True
      self.now_gear = gear

    # Press distance button until we are at the correct bar length. Only change while enabled to avoid skipping startup popup
    if self.frame % 6 == 0 and self.CP.openpilotLongitudinalControl:
      desired_distance = 4 - hud_control.leadDistanceBars
      if CS.out.cruiseState.enabled and CS.pcm_follow_distance != desired_distance:
        self.distance_button = not self.distance_button
      else:
        self.distance_button = 0

    if self.CP.openpilotLongitudinalControl:
      if self.CP.flags & ToyotaFlags.RAISED_ACCEL_LIMIT and self.frame % 3 == 0:
        # internal PCM gas command can get stuck unwinding from negative accel so we apply a generous rate limit
        pcm_accel_cmd = actuators.accel
        if CC.longActive:
          pcm_accel_cmd = rate_limit(pcm_accel_cmd, self.prev_accel, ACCEL_WINDDOWN_LIMIT, ACCEL_WINDUP_LIMIT)
        self.prev_accel = pcm_accel_cmd
        self.deque.append(pcm_accel_cmd)
        self.pcm_accel_cmd.update(pcm_accel_cmd)

        # calculate amount of acceleration PCM should apply to reach target, given pitch
        accel_due_to_pitch = math.sin(self.pitch.x) * ACCELERATION_DUE_TO_GRAVITY
        net_acceleration_request = pcm_accel_cmd + accel_due_to_pitch
        # self.net_acceleration_request.update(net_acceleration_request)

        if CC.longActive and not CS.out.cruiseState.standstill:
          # filter ACCEL_NET so it more closely matches aEgo delay for error correction
          # self.pcm_accel_net.update(CS.pcm_accel_net)

          error = pcm_accel_cmd - CS.out.aEgoBlended
          self.error_rate.update((error - self.prev_error) / (DT_CTRL * 3))

          # self.debug2 = self.error_rate.x
          # self.debug3 = (error - self.prev_error) / (DT_CTRL * 3)

          self.prev_error = error

          # let PCM handle stopping for now
          pcm_accel_compensation = 0.0
          if not stopping:
            self.pid.neg_limit = self.params.ACCEL_MIN - pcm_accel_cmd
            self.pid.pos_limit = self.params.ACCEL_MAX - pcm_accel_cmd

            if self.permit_braking or CS.acc_braking:
              self.last_brake_frame = self.frame

            if (self.frame - self.last_brake_frame) == 2 and pcm_accel_cmd < 0:
              # unwind from diff of CS.pcm_accel_net and pcm_accel_cmd
              self.i = (CS.pcm_accel_net - pcm_accel_cmd)  # * self.pid.k_i  # TODO: try different combos with with override

            # TODO: freeze_integrator when stopping or at standstill?
            #pcm_accel_compensation = self.pid.update(self.deque[round(-40 / 3)] - CS.out.aEgoBlended,
            pcm_accel_compensation = self.pid.update(self.pcm_accel_cmd.x - future_aego,
            # pcm_accel_compensation = self.pid.update(pcm_accel_cmd - future_aego,
                                                     error_rate=self.error_rate.x,  #, feedforward=pcm_accel_cmd)
                                                     freeze_integrator=CS.out.standstill,  # TODO: useless with override
                                                     override=CS.out.standstill)
            pcm_accel_cmd += self.pcm_accel_compensation.update(pcm_accel_compensation)
            # pcm_accel_cmd += pcm_accel_compensation
          else:
            self.pid.reset()
            self.pcm_accel_compensation.x = 0.0

          # pcm_accel_cmd = pcm_accel_cmd + self.pcm_accel_compensation.update(pcm_accel_compensation)

        else:
          self.pid.reset()
          self.pcm_accel_cmd.x = 0.0
          self.pcm_accel_compensation.x = 0.0

        # For cars where we allow a higher max acceleration of 2.0 m/s^2, compensate for PCM request overshoot and imprecise braking
        if False:  #  self.CP.flags & ToyotaFlags.RAISED_ACCEL_LIMIT and CC.longActive and not CS.out.cruiseState.standstill:
          # filter ACCEL_NET so it more closely matches aEgo delay for error correction
          self.pcm_accel_net.update(CS.pcm_accel_net)

          # Our model of the PCM's acceleration request isn't perfect, so we learn the offset when moving
          new_pcm_accel_net = CS.pcm_accel_net
          if stopping or CS.out.standstill:
            # TODO: check if maintaining the offset from before stopping is beneficial
            self.pcm_accel_net_offset.x = 0.0
          else:
            new_pcm_accel_net -= self.pcm_accel_net_offset.update((self.pcm_accel_net.x - accel_due_to_pitch) - CS.out.aEgoBlended)

          # let PCM handle stopping for now, error correct on a delayed acceleration request
          pcm_accel_compensation = 0.0
          if not stopping:
            self.pid.neg_limit = pcm_accel_cmd - self.params.ACCEL_MAX
            self.pid.pos_limit = pcm_accel_cmd - self.params.ACCEL_MIN

            pcm_accel_compensation = self.pid.update(new_pcm_accel_net - self.net_acceleration_request.x)

            # pcm_accel_compensation = 2.0 * (new_pcm_accel_net - self.net_acceleration_request.x)

          # prevent compensation windup
          pcm_accel_compensation = clip(pcm_accel_compensation, pcm_accel_cmd - self.params.ACCEL_MAX,
                                        pcm_accel_cmd - self.params.ACCEL_MIN)

          pcm_accel_cmd = pcm_accel_cmd - self.pcm_accel_compensation.update(pcm_accel_compensation)

        else:
          #self.pcm_accel_compensation.x = 0.0
          self.pcm_accel_net_offset.x = 0.0
          self.net_acceleration_request.x = 0.0
          self.pcm_accel_net.x = CS.pcm_accel_net
          self.permit_braking = True

        # Along with rate limiting positive jerk above, this greatly improves gas response time
        # Consider the net acceleration request that the PCM should be applying (pitch included)
        #net_acceleration_request_min = min(actuators.accel + accel_due_to_pitch, net_acceleration_request)
        net_acceleration_request_min = actuators.accel + accel_due_to_pitch
        if net_acceleration_request_min < 0.1 or stopping or not CC.longActive:
          self.permit_braking = True
        elif net_acceleration_request_min > 0.2:
          self.permit_braking = False

        pcm_accel_cmd = clip(pcm_accel_cmd, self.params.ACCEL_MIN, self.params.ACCEL_MAX)

        can_sends.append(toyotacan.create_accel_command(self.packer, pcm_accel_cmd, pcm_cancel_cmd, self.permit_braking, self.standstill_req, lead,
                                                        CS.acc_type, fcw_alert, self.distance_button))
        self.accel = pcm_accel_cmd

      elif self.frame % 3 == 0:
        actuators_accel = actuators.accel
        comp_thresh = interp(CS.out.vEgo, COMPENSATORY_CALCULATION_THRESHOLD_BP, COMPENSATORY_CALCULATION_THRESHOLD_V)
        # prohibit negative compensatory calculations when first activating long after accelerator depression or engagement
        if not CC.longActive:
          self.prohibit_neg_calculation = True
        # don't reset until a reasonable compensatory value is reached
        if CS.pcm_neutral_force > comp_thresh * self.CP.mass:
          self.prohibit_neg_calculation = False
        # limit minimum to only positive until first positive is reached after engagement, don't calculate when long isn't active
        if CC.longActive and not self.prohibit_neg_calculation:
          accel_offset = CS.pcm_neutral_force / self.CP.mass
        else:
          accel_offset = 0.
        # only calculate pcm_accel_cmd when long is active to prevent disengagement from accelerator depression
        if CC.longActive:
          pcm_accel_cmd = clip(actuators_accel + accel_offset, self.params.ACCEL_MIN, self.params.ACCEL_MAX)
        else:
          pcm_accel_cmd = 0.
        try:
          with open('/tmp/cruise_info.txt','r') as fp:
            cruise_info_str = fp.read()
            if cruise_info_str:
              if cruise_info_str == "1" or cruise_info_str == ",1": #クリープしたければ以下を通さない。
                with open('/tmp/accel_engaged.txt','r') as fp:
                  accel_engaged_str = fp.read()
                  eP_iP = False
                  if int(accel_engaged_str) == 4 and os.path.exists('/tmp/red_signal_eP_iP_set.txt'):
                    with open('/tmp/red_signal_eP_iP_set.txt','r') as fp:
                      red_signal_eP_iP_set_str = fp.read()
                      if red_signal_eP_iP_set_str and int(red_signal_eP_iP_set_str) == 1:
                        eP_iP = True
                  if int(accel_engaged_str) == 3 or eP_iP == True: #ワンペダルモードでもeP(一時的な赤信号手前を除く)では通さない
                    if pcm_accel_cmd > 0:
                      pcm_accel_cmd = 0
                      actuators_accel = 0
        except Exception as e:
          pass
        can_sends.append(toyotacan.create_accel_command_cydia(self.packer, pcm_accel_cmd, actuators_accel, CS.out.aEgo, CC.longActive, pcm_cancel_cmd, self.standstill_req,
                                                        lead, CS.acc_type, fcw_alert, self.distance_button))
        self.accel = pcm_accel_cmd

    else:
      # we can spam can to cancel the system even if we are using lat only control
      if pcm_cancel_cmd:
        if self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
          can_sends.append(toyotacan.create_acc_cancel_command(self.packer))
        elif self.CP.flags & ToyotaFlags.RAISED_ACCEL_LIMIT:
          can_sends.append(toyotacan.create_accel_command(self.packer, 0, pcm_cancel_cmd, True, False, lead, CS.acc_type, False, self.distance_button))
        else:
          can_sends.append(toyotacan.create_accel_command_cydia(self.packer, 0, 0, 0, False, pcm_cancel_cmd, False, lead, CS.acc_type, False, self.distance_button))

    # *** hud ui ***
    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      # ui mesg is at 1Hz but we send asap if:
      # - there is something to display
      # - there is something to stop displaying
      send_ui = False
      if ((fcw_alert or steer_alert) and not self.alert_active) or \
         (not (fcw_alert or steer_alert) and self.alert_active):
        send_ui = True
        self.alert_active = not self.alert_active
      elif pcm_cancel_cmd:
        # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
        send_ui = True

      if self.frame % 20 == 0 or send_ui:
        can_sends.append(toyotacan.create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, hud_control.leftLaneVisible,
                                                     hud_control.rightLaneVisible, hud_control.leftLaneDepart,
                                                     hud_control.rightLaneDepart, CC.enabled, CS.lkas_hud))

      if (self.frame % 100 == 0 or send_ui) and (self.CP.enableDsu or self.CP.flags & ToyotaFlags.DISABLE_RADAR.value):
        can_sends.append(toyotacan.create_fcw_command(self.packer, fcw_alert))

    # *** static msgs ***
    for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
      if self.frame % fr_step == 0 and self.CP.enableDsu and self.CP.carFingerprint in cars:
        can_sends.append(CanData(addr, vl, bus))

    # keep radar disabled
    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      can_sends.append(make_tester_present_msg(0x750, 0, 0xF))

    new_actuators = actuators.as_builder()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel

    new_actuators.debug = self.debug
    new_actuators.debug2 = self.debug2
    new_actuators.debug3 = self.debug3

    self.frame += 1
    return new_actuators, can_sends
