class DCMotor:      


  def __init__(self, fwdPin, revPin) -> None:
    self.fwd=fwdPin
    self.rev=revPin

  def forward(self,pwm) -> None:
    self.rev.duty_u16(65000)
    self.fwd.duty_u16(pwm)

  def backwards(self, pwm) -> None:
    self.rev.duty_u16(pwm)
    self.fwd.duty_u16(65000)

  def stop(self) -> None:
    self.rev.duty_u16(0)
    self.fwd.duty_u16(0)