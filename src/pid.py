


class PID:

    def __init__(self, kp, ki, kd, dt):
        """ Initialise a PID control loop"""

        # Set proportional, integral, derivative factor, and timestamp
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        # Set errors at 0
        self.previousError = 0.0 # previous proportional error
        self.previousIntError = 0.0 # previous integral error

        # Anti wind-up
        self.saturationActivation = False # Boolean to activate saturation for the command
        self.integrateError = True #boolean to choose if we integrate error

        # Used only if saturation is activated
        self.lowSaturation = 0.0
        self.highSaturation = 0.0


    def activateCommandSaturation(self, lowSaturation, highSaturation):
        """
            @brief: Constrain the output PID command between min and max values,
                    and activate conditonal integration anti-windup.
            @param: lowSat : low saturation threshold (= minimum value of command)
            @param: highSat : high saturation threshold (= maximum value of command)
        """
        self.saturationActivation = True
        self.lowSaturation = lowSaturation
        self.highSaturation = highSaturation


    def deactivateCommandSaturation(self):
        """
            @brief: Deactivate PID command saturation and conditonal integration anti-windup.
        """
        self.saturationActivation = False
        self.lowSaturation = 0.0
        self.highSaturation = 0.0
        self.integrateError = True


    def antiWindUp(self, saturation, error):
        """
            @brief: Run conditional anti-windup to prevent integrator term explosion.
                    If the PID output command is saturated, the anti-windup will stop error
                    integration to limit overshoot or divergence.
            @param: error : value of the error (= goalValue - currentValue)
            @param: saturation : value of the PID command saturation (= command - saturatedCommand)
        """

        if (saturation == 0 or error*saturation < 0):
            self.integrateError = True
        else:
            self.integrateError = False

    def computeCommand(self, error):
        """
            @brief: Run the PID control
            @param: error : value of the error (= goalValue - currentValue)
            @return: computed PID control value
        """
        # compute PID command
        command = self.kp*error +\
                  (self.kd/self.dt)*(error - self.previousError) +\
                  self.ki*self.dt*self.previousIntError

        # saturate command PID if enabled
        if self.saturationActivation:
            # compute saturated command
            saturatedCommand = max(min(command, self.highSaturation), self.lowSaturation)

            # Use anti-windup
            self.antiWindUp(error, command - saturatedCommand)

            # Update command with saturated
            command = saturatedCommand

        # Update error
        self.previousError = error

        # if integral error activated, compute it
        if self.integrateError:
            self.previousIntError += error

        return command
