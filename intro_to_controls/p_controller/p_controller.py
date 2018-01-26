# p_controller.py
# A proportional controller for a quad copter that controls the control
# effort depending on the error ferom a reference, as a function of time

class P_Controller:

    def __init__(self, kp=0.0, start_time=0):
        self.kp_ = float(kp)

        self.set_point_ = 0.0   # desired value

        self.start_time_ = start_time

        self.last_timestamp_ = 0

        # Control effort history
        self.u_p = [0]


    # set the desired value set_point
    def setTarget(self, target):
        self.set_point_ = float(target)

    # set the proportional gain value
    def setKP(self, kp_value):
        self.kp_ = float(kp_value)

    def update(self, measured_value, timestamp):
        # Calculate delta_time using the last_timestamp_
        # and the provided timestamp argument
        ########################################
        delta_time = timestamp - self.last_timestamp_
        ########################################
        
        if delta_time == 0:
            # Delta time is zero
            return 0
        
        # Calculate the error as the differnce between
        # the set_point_ and the measured_value
        ########################################
        error = self.set_point_ - measured_value
        ########################################
        
        # Set the last_timestamp_ to current timestamp
        ########################################
        self.last_timestamp_ = timestamp
        ########################################

        # Calculate the proportional error here. Be sure to access the 
        # the internal Kp class variable
        ########################################
        p = self.kp_ * error
        ########################################

        # Set the control effort
        # u is the sum of all your errors. In this case it is just 
        # the proportional error.
        ########################################
        u = p
        ########################################
        
        # Here we are storing the control effort history for post control
        # observations. 
        self.u_p.append(p)

        return u


