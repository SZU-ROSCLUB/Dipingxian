"""Small PID controller used by the steering loop."""


class PID:
    """Discrete PID controller with integral and output limits."""

    def __init__(self, dt):
        if dt <= 0.0:
            raise ValueError('PID dt must be greater than zero')

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.dt = float(dt)

        self.reference = 0.0
        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.output = 0.0

        self.integral_limit = 0.0
        self.output_limit = 0.0

    def set_kp(self, kp):
        """Set the proportional gain."""

        self.kp = float(kp)

    def set_ki(self, ki):
        """Set the integral gain."""

        self.ki = float(ki)

    def set_kd(self, kd):
        """Set the derivative gain."""

        self.kd = float(kd)

    def set_dt(self, dt):
        """Set the sample time after validating it."""

        if dt <= 0.0:
            raise ValueError('PID dt must be greater than zero')
        self.dt = float(dt)

    def set_integral_limit(self, limit):
        """Set the symmetric integral clamp."""

        self.integral_limit = abs(float(limit))

    def set_output_limit(self, limit):
        """Set the symmetric output clamp."""

        self.output_limit = abs(float(limit))

    def update(self, reference, actual):
        """Update the controller from a reference and actual value."""

        self.reference = float(reference)
        return self.update_error(self.reference - float(actual))

    def update_error(self, error):
        """Update the controller from a precomputed error."""

        self.error = float(error)
        self.integral += self.error * self.dt
        self.integral = max(
            min(self.integral, self.integral_limit),
            -self.integral_limit,
        )
        self.derivative = (self.error - self.prev_error) / self.dt
        self.output = (
            self.kp * self.error
            + self.ki * self.integral
            + self.kd * self.derivative
        )
        self.prev_error = self.error
        self.output = float(
            max(
                min(self.output, self.output_limit),
                -self.output_limit,
            )
        )
        return self.output

    def reset(self):
        """Clear all accumulated controller state."""

        self.reference = 0.0
        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.output = 0.0

    # Compatibility aliases for the previous public API.
    setKp = set_kp
    setKi = set_ki
    setKd = set_kd
    setDt = set_dt
    setIntegralLimit = set_integral_limit
    setOutputLimit = set_output_limit
    update_err = update_error
