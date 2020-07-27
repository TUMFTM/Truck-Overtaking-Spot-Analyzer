import math
import numpy as np


class VehicleState:

    def __init__(self, position, set_velocity, vel_high, vel_low, vehicle_mass):
        """The vehicle state holds the dynamic and static variables of the vehicle"""
        # Static variables
        self.mass = vehicle_mass
        self.length = 16
        self.set_velocity = set_velocity
        self.velocity_low = self.set_velocity - vel_low
        self.velocity_high = self.set_velocity + vel_high
        self.desired_velocity = set_velocity

        # Dynamic variable
        self.position = position
        self.velocity = self.set_velocity
        self.acceleration = 0
        self.fuel = 0
        self.power = 0
        self.desired_acceleration = 0
        self.state = 0


class Engine:

    def __init__(self):
        """Sets the properties for a heavy duty truck"""
        # Power properties
        self.p_min = 25000  # [W] Minimal power the engine can provide
        self.p_max = 350000  # [W] Maximal power the engine can provide
        self.p_ideal = 200000
        self.efficiency = 0.92  # [%]  Fraction of energy that is really available for movement

        # Driving resistances
        self.c_d = 0.44  # [-]     Drag coefficient
        self.c_r = 0.007  # [-]     Rolling resistance coefficient
        self.roh_a = 1.212  # [kg/m³] Air density
        self.g = 9.81  # [m/s²]  Gravitational acceleration
        self.A_f = 8.4  # [m²]    Front surface

        # Fuel consumption
        self.g_to_ml = 1000 / 830  # [ml/g] conversion from grams to milliliter
        power = list(range(25, 351, 25))
        fuel_g_per_wh = [230, 210, 200, 195, 191, 188, 186, 185, 186, 188, 190, 193, 195, 200]
        # [g/kWh] calculates fuel depending on power
        self.fuel_function = np.poly1d(np.polyfit(power, fuel_g_per_wh, 5))

    def calculate_required_power(self, vehicle, a, v=None, slope=None):
        """calculated the required power to generate a specified acceleration"""
        m = vehicle.state.mass
        if v is None:
            v = vehicle.state.velocity
        if v == 0:
            v = 1
        F_dr = a * m  # Driving force
        F_tot = self.calculate_total_driving_resistance(vehicle, v, slope)
        P_dr = (F_dr + F_tot) * v  # Driving Power
        P = P_dr / self.efficiency  # Power provided by the engine

        if P <= 0:
            return 0
        P = self.get_power_within_limitation(P)
        return P

    def apply_power(self, vehicle, P, v=None, slope=None):
        """Applies a specified Power and returns the resulting acceleration and the required fuel"""
        m = vehicle.state.mass
        if v is None:
            v = vehicle.state.velocity
        if v == 0:
            v = 1
        P = self.get_power_within_limitation(P)
        P_dr = P * self.efficiency
        F_tot = self.calculate_total_driving_resistance(vehicle, v, slope)
        F_dr = (P_dr / v) - F_tot
        a = F_dr / m

        if P > self.p_min:
            # fuel in ml/s
            return a, self.fuel_function(P / 1000) * (P / 1000) * (1 / 3600) * self.g_to_ml
        else:
            return a, 0

    def calculate_total_driving_resistance(self, vehicle, v, slope):
        """Calculates the total driving resistance"""
        a = vehicle.state.acceleration
        m = vehicle.state.mass
        if v is None:
            v = vehicle.state.velocity
        if slope is None:
            slope = vehicle.road_traffic.road.slope[int(vehicle.state.position / 50)]
        alpha = math.atan(slope / 100)

        F_r = m * self.g * self.c_r * math.cos(alpha)  # Rolling resistance force
        F_a = 0.5 * self.roh_a * self.c_d * self.A_f * v ** 2  # Air resistance force
        F_g = m * self.g * math.sin(alpha)  # Slope resistance force
        F_m = 0  # m * a  # Acceleration resistance force
        F_tot = F_r + F_a + F_g + F_m  # Total driving resistance

        return F_tot

    def get_power_within_limitation(self, P):
        """Sets a power value within the power limits of the engine"""
        if P > self.p_max:
            P = self.p_max
        elif P < 0:
            P = 0
        elif 0 < P < self.p_min:
            P = self.p_min

        return P

    def get_acceleration_zero_power(self, vehicle, v=None, slope=None):
        """Calculates the acceleration with zero power at the engine"""
        return self.apply_power(vehicle, 0, v, slope)

    def get_acceleration_ideal_power(self, vehicle, v=None, slope=None):
        """Calculates the acceleration when the engine is working at its most efficient point"""
        return self.apply_power(vehicle, self.p_ideal, v, slope)

    def get_acceleration_max_power(self, vehicle, v=None, slope=None):
        """Calculates the acceleration when the engine is working at its maximal power"""
        return self.apply_power(vehicle, self.p_max, v, slope)


class VelocityProfile:

    def __init__(self, vehicle, slope, start_vel=-1):
        self.slope = slope
        self.vehicle = vehicle

        if start_vel == -1:
            start_vel = self.vehicle.state.set_velocity
            while self.vehicle.engine.calculate_required_power(self.vehicle, 0, start_vel,
                                                               self.slope[0]) >= self.vehicle.engine.p_max:
                start_vel -= 0.2

        self.velocity_profile = self.calculate_velocity_profile(0, len(slope) * 50, start_vel)
        # Sometimes one 10m element ist missing due to discretisation. In that case, duplicate the last element.
        if len(self.velocity_profile) < len(slope) * 50 / 10:
            self.velocity_profile.append(self.velocity_profile[-1])
        assert len(self.velocity_profile) == len(slope) * 50 / 10

    def get_desired_velocity(self, pos=None):
        """Returns the value of the desired velocity profile at a specified position"""
        if pos is None:
            pos = self.vehicle.state.position
        return self.velocity_profile[int(pos / 10)][1]

    def get_velocity(self, pos=None):
        """Returns the value of the velocity profile at a specified position"""
        if pos is None:
            pos = self.vehicle.state.position
        return self.velocity_profile[int(pos / 10)][0]

    def calculate_velocity_profile(self, start_pos, end_pos, start_vel):
        """Core function to start calculating the velocity profile"""
        if end_pos - start_pos < 10:
            return []
        slope = self.slope
        slope = slope[int(start_pos / 50):int(end_pos / 50)]
        if slope[0] == 0:
            # Right now it is flat, in future it can stay flat, go up or go down
            change = 0
            dist_to_change = 0
            for s in slope:
                if s > 0:
                    change = 1
                    dist_to_change = (slope.index(s)) * 50
                    break
                elif s < 0:
                    change = -1
                    dist_to_change = (slope.index(s)) * 50
                    break

            if change == 0:
                return self.calculate_velocity_profile_flat(start_pos, end_pos, start_vel)
            elif change < 0:
                first_segment = self.calculate_velocity_profile_flat(start_pos, start_pos + dist_to_change, start_vel)
                next_segment = self.calculate_velocity_profile(start_pos + dist_to_change, end_pos,
                                                               first_segment[-1][0])
                return first_segment + next_segment
            elif change > 0:
                first_segment = self.calculate_velocity_profile_flat_to_uphill(start_pos, start_pos + dist_to_change,
                                                                               start_vel)
                next_segment = self.calculate_velocity_profile(start_pos + dist_to_change, end_pos,
                                                               first_segment[-1][0])
                return first_segment + next_segment
        elif slope[0] < 0:
            change = 0
            dist_to_change = 0
            for s in slope:
                if s >= 0:
                    change = 1
                    dist_to_change = (slope.index(s)) * 50
                    break
            if change == 0:
                return self.calculate_velocity_profile_downhill(start_pos, end_pos, start_vel)
            else:
                first_segment = self.calculate_velocity_profile_downhill(start_pos, start_pos + dist_to_change,
                                                                         start_vel)
                next_segment = self.calculate_velocity_profile(start_pos + dist_to_change, end_pos,
                                                               first_segment[-1][0])
                return first_segment + next_segment
        elif slope[0] > 0:
            change = 0
            dist_to_change = 0
            for s in slope:
                if s <= 0:
                    change = 1
                    dist_to_change = (slope.index(s)) * 50
                    break
            if change == 0:
                return self.calculate_velocity_profile_uphill(start_pos, end_pos, start_vel)
            else:
                first_segment = self.calculate_velocity_profile_uphill_to_lower(start_pos, start_pos + dist_to_change,
                                                                                start_vel)
                next_segment = self.calculate_velocity_profile(start_pos + dist_to_change, end_pos,
                                                               first_segment[-1][0])
                return first_segment + next_segment

    def calculate_velocity_profile_flat(self, start_pos, end_pos, start_vel):
        """Calculates the velocity profile on a flat segment"""
        vel_set = self.vehicle.state.set_velocity
        pos = start_pos
        vel = start_vel
        vel_profile = []

        while pos < end_pos:
            if vel < vel_set - 0.1:
                a, _ = self.vehicle.engine.get_acceleration_ideal_power(self.vehicle, vel, slope=0)
                vel += a
            elif vel_set - 0.1 <= vel <= vel_set + 0.1:
                vel = vel_set
            elif vel > vel_set + 0.1:
                a, _ = self.vehicle.engine.get_acceleration_zero_power(self.vehicle, vel, slope=0)
                vel += a
            pos += vel
            vel_profile.append([vel, vel_set])
        return VelocityProfile.velocity_over_time_to_distance(vel_profile, end_pos - start_pos)

    def calculate_velocity_profile_uphill(self, start_pos, end_pos, start_vel):
        """Calculates the velocity profile on an uphill segment"""
        pos = start_pos
        vel = start_vel
        vel_profile = []

        while pos < end_pos:
            if vel <= self.vehicle.state.set_velocity + 0.1:
                required_power = self.vehicle.engine.calculate_required_power(self.vehicle,
                                                                              a=self.vehicle.state.set_velocity - vel,
                                                                              v=vel,
                                                                              slope=self.slope[int(pos / 50)])
                a, _ = self.vehicle.engine.apply_power(self.vehicle, P=required_power, v=vel,
                                                       slope=self.slope[int(pos / 50)])
                vel += a
            elif vel > self.vehicle.state.set_velocity + 0.1:
                a, _ = self.vehicle.engine.get_acceleration_zero_power(self.vehicle, vel,
                                                                       slope=self.slope[int(pos / 50)])
                vel += a
            pos += vel
            vel_profile.append([vel, self.vehicle.state.set_velocity])

        return VelocityProfile.velocity_over_time_to_distance(vel_profile, end_pos - start_pos)

    def calculate_velocity_profile_uphill_to_lower(self, start_pos, end_pos, start_vel):
        """Calculates the velocity profile on a uphill to lower segment"""
        # At the end_pos the vehicle should drive at most with v_set - h_minus
        pos = start_pos
        vel = start_vel

        vel_profile = []
        while pos < end_pos:
            if vel <= self.vehicle.state.velocity_low:
                required_power = self.vehicle.engine.calculate_required_power(self.vehicle,
                                                                              a=self.vehicle.state.set_velocity - vel,
                                                                              v=vel,
                                                                              slope=self.slope[int(pos / 50)])
                a, _ = self.vehicle.engine.apply_power(self.vehicle, P=required_power, v=vel,
                                                       slope=self.slope[int(pos / 50)])
                vel += a
            elif self.vehicle.state.velocity_low < vel <= self.vehicle.state.set_velocity + 0.1:
                # check if rolling down to vel_set-vel_minus brings the truck to the end_position
                roll_profile = self.calculate_velocity_profile_rolling(pos, end_pos, vel)
                if pos + len(roll_profile) * 10 >= end_pos:
                    vel_profile = VelocityProfile.velocity_over_time_to_distance(vel_profile)
                    vel_profile = vel_profile + roll_profile
                    vel_profile = vel_profile[:int((end_pos - start_pos) / 10)]
                    return vel_profile
                else:
                    required_power = self.vehicle.engine.calculate_required_power(self.vehicle,
                                                                                  a=self.vehicle.state.set_velocity - vel,
                                                                                  v=vel,
                                                                                  slope=self.slope[int(pos / 50)])
                    a, _ = self.vehicle.engine.apply_power(self.vehicle, P=required_power, v=vel,
                                                           slope=self.slope[int(pos / 50)])
                    vel += a
            elif vel > self.vehicle.state.set_velocity + 0.1:
                a, _ = self.vehicle.engine.get_acceleration_zero_power(self.vehicle, vel,
                                                                       slope=self.slope[int(pos / 50)])
                vel += a
            pos += vel
            vel_profile.append([vel, self.vehicle.state.set_velocity])
        return VelocityProfile.velocity_over_time_to_distance(vel_profile, end_pos - start_pos)

    def calculate_velocity_profile_downhill(self, start_pos, end_pos, start_vel):
        """Calculates the velocity profile on a downhill segment"""
        pos = start_pos
        vel = start_vel

        vel_profile = []
        if start_vel > self.vehicle.state.velocity_high:
            over_high = True
        else:
            over_high = False

        while pos < end_pos:
            if vel < self.vehicle.state.velocity_low:
                a, _ = self.vehicle.engine.get_acceleration_ideal_power(self.vehicle, vel,
                                                                        slope=self.slope[int(pos / 50)])
                vel += a
            else:
                # velocity is equal or above the low vel limit
                roll_acc, _ = self.vehicle.engine.get_acceleration_zero_power(self.vehicle, vel,
                                                                              slope=self.slope[int(pos / 50)])

                ########################################################################################################
                # Changed Code # Start #################################################################################
                ########################################################################################################
                if vel > self.vehicle.state.velocity_high:
                    # velocity is above high vel -> reduce velocity either by rolling or braking
                    roll_acc = min(roll_acc, -0.1)
                    vel += roll_acc
                    if not over_high:
                        vel = min(vel, self.vehicle.state.velocity_high)

                elif vel >= self.vehicle.state.set_velocity:
                    over_high = False
                    # velocity is between set and high vel -> just roll
                    vel += roll_acc
                    vel = min(vel, self.vehicle.state.velocity_high)
                else:
                    # vel is between low and set vel -> speed up either by rolling or engine
                    over_high = False
                    if roll_acc > 0:
                        vel += roll_acc
                        vel = min(vel, self.vehicle.state.velocity_high)

                    else:
                        a, _ = self.vehicle.engine.get_acceleration_ideal_power(self.vehicle, vel,
                                                                                slope=self.slope[int(pos / 50)])
                        vel += a
                        vel = min(vel, self.vehicle.state.velocity_high)

                ########################################################################################################
                # Changed Code # End ###################################################################################
                ########################################################################################################

            pos += vel
            vel_profile.append([vel, self.vehicle.state.velocity_high])

        return VelocityProfile.velocity_over_time_to_distance(vel_profile, end_pos - start_pos)

    def calculate_velocity_profile_flat_to_uphill(self, start_pos, end_pos, start_vel):
        """Calculates the velocity profile on a flat to uphill segment"""
        pos = start_pos
        vel = start_vel
        vel_profile = []

        while pos < end_pos:
            if vel < self.vehicle.state.velocity_high:
                # only accelerate if vset + vplus is not reached
                vel_profile_acceleration = \
                    self.calculate_velocity_profile_acceleration_on_flat(pos, vel, self.vehicle.state.velocity_high)
                acceleration_length = len(vel_profile_acceleration) * 10
                if acceleration_length >= end_pos - pos:
                    flat_segment = VelocityProfile.velocity_over_time_to_distance(vel_profile, pos - start_pos)
                    flat_to_uphill_profile = flat_segment + vel_profile_acceleration
                    return flat_to_uphill_profile[0:int((end_pos - start_pos) / 10)]

            if vel < self.vehicle.state.set_velocity - 0.1:
                a, _ = self.vehicle.engine.get_acceleration_ideal_power(self.vehicle, vel, slope=0)
                vel += a
            elif self.vehicle.state.set_velocity - 0.1 <= vel <= self.vehicle.state.set_velocity + 0.1:
                vel = self.vehicle.state.set_velocity
            elif vel > self.vehicle.state.set_velocity + 0.1:
                a, _ = self.vehicle.engine.get_acceleration_zero_power(self.vehicle, vel, slope=0)
                vel += a
            pos += vel
            vel_profile.append([vel, self.vehicle.state.set_velocity])

        return VelocityProfile.velocity_over_time_to_distance(vel_profile, end_pos - start_pos)

    def calculate_velocity_profile_acceleration_on_flat(self, start_pos, start_vel, vel_goal):
        """Calculates the velocity with efficient acceleration on a flat segment"""
        pos = start_pos
        vel = start_vel
        vel_profile = []

        while vel < vel_goal:
            a, _ = self.vehicle.engine.get_acceleration_ideal_power(self.vehicle, vel, 0)
            vel += a
            pos += vel
            vel_profile.append([vel, vel_goal])
        return VelocityProfile.velocity_over_time_to_distance(vel_profile)

    def calculate_velocity_profile_rolling(self, start_pos, end_pos, start_vel):
        """Calculates the velocity profile with only rolling"""
        vel = start_vel
        pos = start_pos
        vel_profile = []

        while vel >= self.vehicle.state.velocity_low - 0.1 and pos < end_pos:
            a, _ = self.vehicle.engine.get_acceleration_zero_power(self.vehicle, v=vel, slope=self.slope[int(pos / 50)])
            vel += a
            vel_profile.append([vel, self.vehicle.state.velocity_low])
            pos += vel

        return VelocityProfile.velocity_over_time_to_distance(vel_profile)

    @staticmethod
    def velocity_over_time_to_distance(vel_over_time, length=None):
        """Converts the velocity profile over time to a velocity profile over distance"""
        if vel_over_time == []:
            return []
        dist = [vel_over_time[0][0]]
        for i in range(1, len(vel_over_time)):
            dist.append(dist[i - 1] + vel_over_time[i][0])
        vel_over_dist = []
        current_index = 0
        if length is None:
            length = int(dist[-1] // 10 * 10)
        for i in range(0, int(length), 10):
            while i > dist[current_index]:
                current_index += 1
            vel_over_dist.append(vel_over_time[current_index])
        return vel_over_dist


class Vehicle:

    def __init__(self, state, slope, start_vel=-1):
        self.state = state
        self.engine = Engine()
        self.velocity_profile = VelocityProfile(self, slope, start_vel)
