import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.values import CarControllerParams, TeslaFlags
from opendbc.car.vehicle_model import VehicleModel

# MADS（エンゲージせずに操舵している間）だけ、ウインカー・ハザードが点いている間と消えてから
# この秒数は操舵しない。点滅の消えている間も止めたままにするため（オーナーの controlsd.py の移植）
MADS_BLINKER_HOLD = 1.0  # s

# 低速での目標角度の整え（オーナーの latcontrol_angle.py の移植）。低速では曲率から角度への変換で
# モデルの揺れが大きくなり、EPS が震えるので、速度で薄れる一次遅れと、速度で緩む変化の上限をかける
SMOOTH_TAU_LOW_SPEED = 0.30   # s, SMOOTH_SPEED_LO 以下
SMOOTH_SPEED_LO = 3.0         # m/s
SMOOTH_SPEED_HI = 12.0        # m/s, これ以上で tau = 0
RATE_LIMIT_LOW_SPEED = 60.0   # deg/s, SMOOTH_SPEED_LO 以下
RATE_LIMIT_HIGH_SPEED = 250.0  # deg/s, RATE_SPEED_HI 以上
RATE_SPEED_HI = 15.0          # m/s


def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  # A Model 3 at 40 m/s using the Model Y limits sees a <0.3% difference in max angle (from curvature factor)
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(CP, self.packer)

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

    # Cooperative steering: LANE_KEEP_ASSIST commands instead of ANGLE_CONTROL
    self.control_type = 2 if (CP.flags & TeslaFlags.COOP_STEERING) else 1

    self.mads_blinker_hold = 0.0
    self.angle_des_filt = 0.0

  def _condition_angle(self, raw_angle_des: float, v_ego: float) -> float:
    prev = self.angle_des_filt
    tau = float(np.interp(v_ego, [SMOOTH_SPEED_LO, SMOOTH_SPEED_HI], [SMOOTH_TAU_LOW_SPEED, 0.0]))
    alpha = DT_CTRL / (DT_CTRL + tau) if tau > 0.0 else 1.0
    filtered = prev + alpha * (raw_angle_des - prev)
    max_step = float(np.interp(v_ego, [SMOOTH_SPEED_LO, RATE_SPEED_HI], [RATE_LIMIT_LOW_SPEED, RATE_LIMIT_HIGH_SPEED])) * DT_CTRL
    self.angle_des_filt = prev + float(np.clip(filtered - prev, -max_step, max_step))
    return self.angle_des_filt

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # Tesla EPS enforces disabling steering on heavy lateral override force.
    # When enabling in a tight curve, we wait until user reduces steering force to start steering.
    # Canceling is done on rising edge and is handled generically with CC.cruiseControl.cancel
    # With STEER_NO_DISENGAGE there is no disengage to wait for; controlsd already drops
    # latActive while the driver is overriding (steeringPressed), so no extra gate is needed
    lat_active = CC.latActive and (CS.hands_on_level < 3 or bool(self.CP.flags & TeslaFlags.STEER_NO_DISENGAGE))

    # エンゲージ中（CC.enabled）は変えない。MADS だけで操舵しているときに、ウインカー中は止める
    if CS.out.leftBlinker or CS.out.rightBlinker:
      self.mads_blinker_hold = MADS_BLINKER_HOLD
    elif self.mads_blinker_hold > 0.0:
      self.mads_blinker_hold -= DT_CTRL
    if not CC.enabled and self.mads_blinker_hold > 0.0:
      lat_active = False

    # 毎フレーム（controlsd と同じ 100Hz）で整える。操舵していない間は今の角度に合わせ、始めた瞬間に飛ばないようにする
    if lat_active:
      steering_angle_des = self._condition_angle(actuators.steeringAngleDeg, CS.out.vEgo)
    else:
      self.angle_des_filt = CS.out.steeringAngleDeg
      steering_angle_des = actuators.steeringAngleDeg

    if self.frame % 2 == 0:
      # Angular rate limit based on speed
      self.apply_angle_last = apply_steer_angle_limits_vm(steering_angle_des, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)

      can_sends.append(self.tesla_can.create_steering_control(self.apply_angle_last, lat_active, self.control_type))

    if self.frame % 10 == 0:
      can_sends.append(self.tesla_can.create_steering_allowed())

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        state = 13 if CC.cruiseControl.cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))

    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if CC.cruiseControl.cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
