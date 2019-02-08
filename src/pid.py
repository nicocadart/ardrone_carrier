class PID:

    def __init__(self, kp, ki, kd, dt):
        """
        Initialise a PID control loop
        """
        # Set proportional, integral, derivative factor, and control period
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        # Set errors at 0
        self.previous_error = 0.0  # previous proportional error
        self.previous_integral_error = 0.0  # previous integral error

        # Anti wind-up
        self.saturation_activation = False  # flag to activate saturation for the command
        self.integrate_error = True  # flag to choose if we integrate error

        # Used only if saturation is activated
        self.low_saturation = 0.0
        self.high_saturation = 0.0

    def activate_command_saturation(self, low_saturation, high_saturation):
        """
        @brief: Constrain the output PID command between min and max values,
                and activate conditonal integration anti-windup.
        @param: lowSat : low saturation threshold (= minimum value of command)
        @param: highSat : high saturation threshold (= maximum value of command)
        """
        self.saturation_activation = True
        self.low_saturation = low_saturation
        self.high_saturation = high_saturation

    def deactivate_command_saturation(self):
        """
        @brief: Deactivate PID command saturation and conditonal integration anti-windup.
        """
        self.saturation_activation = False
        self.low_saturation = 0.0
        self.high_saturation = 0.0
        self.integrate_error = True

    def anti_windup(self, error, saturation):
        """
        @brief: Run conditional anti-windup to prevent integrator term explosion.
                If the PID output command is saturated, the anti-windup will stop error
                integration to limit overshoot or divergence.
        @param: error : value of the error (= goalValue - currentValue)
        @param: saturation : value of the PID command saturation (= command - saturatedCommand)
        """
        if saturation == 0 or error * saturation < 0:
            self.integrate_error = True
        else:
            self.integrate_error = False

    def compute_command(self, error):
        """
        @brief: Run the PID control
        @param: error : value of the error (= goalValue - currentValue)
        @return: computed PID control value
        """
        # compute PID command
        command = self.kp * error + \
                  (self.kd / self.dt) * (error - self.previous_error) + \
                  self.ki * self.dt * self.previous_integral_error

        # saturate command PID if enabled
        if self.saturation_activation:
            # compute saturated command
            saturatedCommand = max(min(command, self.high_saturation), self.low_saturation)

            # Use anti-windup
            self.anti_windup(error, command - saturatedCommand)

            # Update command with saturated
            command = saturatedCommand

        # Update error
        self.previous_error = error

        # if integral error activated, compute it
        if self.integrate_error:
            self.previous_integral_error += error

        return command
