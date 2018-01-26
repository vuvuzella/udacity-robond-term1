##################################################################################
# Your goal is to follow the comments and complete the the tasks asked of you.
#
# Good luck designing your proportional derivative controller!
#
##################################################################################

class PD_Controller:
    def __init__(self, kp = 0.0, kd = 0.0, start_time = 0):
        
        # The PD controller can be initalized with a specific kp value
        # and kd value
        self.kp_ = float(kp)
        self.kd_ = float(kd)
        
        # Define last_error_ and set to 0.0
        ########################################
        self.last_error_ = 0.0
        ########################################

        # Store relevant data
        self.last_timestamp_ = 0.0
        self.set_point_ = 0.0
        self.start_time_ = start_time
        self.error_sum_ = 0.0

        # Control effort history
        self.u_p = [0]
        self.u_d = [0]

    def setTarget(self, target):
        self.set_point_ = float(target)

    def setKP(self, kp):
        self.kp_ = float(kp)
        
    def setKD(self, kd):
        # Set the internal kd_ value with the provided variable
        ########################################
        self.kd_ = float(kd)
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

        # Find error_sum_
        self.error_sum_ += error * delta_time
        
        # Calculate the delta_error
        ########################################
        delta_error = (error - self.last_error_)
        ########################################
        
        # Update the past error with the current error
        ########################################
        self.last_error_ = error
        ########################################

        # Proportional error
        p = self.kp_ * error
       
        # Calculate the derivative error here. Be sure to access the 
        # the internal Kd class variable
        ########################################
        d = self.kd_ * (delta_error / delta_time)
        ########################################
        
        # Set the control effort
        # u is the sum of all your errors. In this case it is just 
        # the proportional and derivative error.
        ########################################
        u = p + d
        ########################################
        
        # Here we are storing the control effort history for post control
        # observations. 
        self.u_p.append(p)
        self.u_d.append(d)

        return u
