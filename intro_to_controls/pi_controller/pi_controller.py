##################################################################################
# Your goal is to follow the comments and complete the the tasks asked of you.
#
# Good luck designing your proportional integral controller!
#
##################################################################################


class PI_Controller:
    def __init__(self, kp = 0.0, ki = 0.0, start_time = 0):
        
        # The PI controller can be initalized with a specific kp value
        # and ki value
        self.kp_ = float(kp)
        self.ki_ = float(ki)

        # Define error_sum_ and set to 0.0
        ########################################
        self.error_sum_ = 0.0
        ########################################

        # Store relevant data
        self.last_timestamp_ = 0.0
        self.set_point_ = 0.0
        self.start_time_ = start_time

        # Control effort history
        self.u_p = [0]
        self.u_i = [0]

    def setTarget(self, target):
        self.set_point_ = float(target)

    def setKP(self, kp):
        self.kp_ = float(kp)
        
    def setKI(self, ki):
        # Set the internal ki_ value with the provided variable
        ########################################
        self.ki_ = ki
        ########################################

    def update(self, measured_value, timestamp):
        delta_time = timestamp - self.last_timestamp_
        if delta_time == 0:
            # Delta time is zero
            return 0
        
        # Calculate the error 
        error = self.set_point_ - measured_value
        
        # Set the last_timestamp_ 
        self.last_timestamp_ = timestamp

        # Calculate the error_sum_
        ########################################
        self.error_sum_ = self.error_sum_ + (error * delta_time)
        ########################################
        
        # Proportional error
        p = self.kp_ * error
       
        # Calculate the integral error here. Be sure to access the 
        # the internal Ki class variable
        ########################################
        i = self.ki_ * self.error_sum_
        ########################################
        
        # Set the control effort
        # u is the sum of all your errors. In this case it is just 
        # the proportional and integral error.
        ########################################
        u = p + i
        ########################################
        
        # Here we are storing the control effort history for post control
        # observations. 
        self.u_p.append(p)
        self.u_i.append(i)

        return u
