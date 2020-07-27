import math
import multiprocessing as mp
import queue
import time

from Vehicle import Vehicle, VehicleState


# TODO Make more realistic Stvo maneuver
# TODO Within 'None' Maneuver Truck A does not adapt Truck B vel on steep uphill slopes

class OvertakingSpotAnalyzer:

    @staticmethod
    def run_analysis(slope, truck_a, truck_b, step_size, cost_parameter):
        """Core method of the osa thar runs the full analysis"""
        start = time.clock()
        OvertakingSpotAnalyzer.add_trucks_with_v_profile(slope, truck_a, truck_b)

        first_contact = OvertakingSpotAnalyzer.calculate_contact(truck_a['v_profile'], truck_a['start'],
                                                                 truck_b['v_profile'], truck_b['start'])

        pc_details, ac_details, overtaking = OvertakingSpotAnalyzer.calculate_overtaking(slope, truck_a, truck_b,
                                                                                         step_size, first_contact)

        OvertakingSpotAnalyzer.asses_and_select_overtaking_spots(pc_details, ac_details, overtaking, cost_parameter)

        end = time.clock()

        return {'truck_a': truck_a, 'truck_b': truck_b, 'first_contact': first_contact,
                'pc_details': pc_details, 'ac_details': ac_details, 'overtaking': overtaking,
                'calculation_time': end - start}

    @staticmethod
    def add_trucks_with_v_profile(slope, truck_a, truck_b):
        """Adds the generated trucks and velocity profiles to the dict

        Args:
            slope: The slope over the full road
            truck_a: Truck A dictionary
            truck_b: Truck B dictionary

        Note:
            This function does not return anything.
            But it adds the truck and velocity profile to the existing dicts
        """
        a_truck, a_v_profile = OvertakingSpotAnalyzer.get_truck_with_profile(slope, truck_a['vel'], truck_a['weight'],
                                                                             truck_a['vel_p'], truck_a['vel_m'])

        _, a_max_v_profile = OvertakingSpotAnalyzer.get_truck_with_profile(slope, truck_a['vel'] + truck_a['vel_p'],
                                                                           truck_a['weight'])

        b_truck, b_v_profile = OvertakingSpotAnalyzer.get_truck_with_profile(slope, truck_b['vel'], truck_b['weight'],
                                                                             truck_b['vel_p'], truck_b['vel_m'])

        b_min_v_profile = [min(e, (truck_b['vel'] + truck_b['vel_m']) / 3.6) for e in b_v_profile]

        truck_a['truck'] = a_truck
        truck_a['v_profile'] = a_v_profile
        truck_a['v_profile_max'] = a_max_v_profile

        truck_b['truck'] = b_truck
        truck_b['v_profile'] = b_v_profile
        truck_b['v_profile_min'] = b_min_v_profile

    @staticmethod
    def get_truck_with_profile(slope, set_vel, mass, v_plus=0, v_minus=0, start_v=-1, start_i=-1):
        """Instantiates the truck from vehicle class

        Args:
            slope: The slope over the full road
            set_vel (int): Set velocity for the GPS-Cruise-Control
            mass (int): The total vehicle mass
            v_plus (int): The positive hysteresis for the GPS-Cruise-Control
            v_minus (int): The negative hysteresis for the GPS-Cruise-Control
            start_v: The start velocity of the truck
            start_i: The index of the road at which the truck starts

        Returns:
            truck: Instance of vehicle
            v_profile: Output of the trucks GPS-Cruise-Control"""
        state = VehicleState(0, set_vel / 3.6, v_plus / 3.6, v_minus / -3.6, mass * 1000)

        if start_v > -1 and start_i > -1:
            truck = Vehicle(state, slope[(start_i * 10) // 50:], start_v)
        else:
            truck = Vehicle(state, slope)

        v_profile = [e[0] for e in truck.velocity_profile.velocity_profile]
        return truck, v_profile

    @staticmethod
    def calculate_contact(a_v_profile, a_start, b_v_profile, b_start, pre=True):
        """Calculates time and position when two trucks meet, i.e. reach the 50m safety gap

        Note:
            Can be used to calculate when and where the overtaker reaches the slower moving truck.
            But also when and where the fast truck passed the slower truck.

        Args:
            a_v_profile: Velocity Profile of Truck A
            a_start: Start Position of Truck A
            b_v_profile: Velocity Profile of Truck B
            b_start: Start Position of Truck B
            pre (bool): True to calculate the contact before overtaking, False to calculate contact after overtaking

        Returns: Returns a dict containing details about when and where the trucks meet
        """
        dist = OvertakingSpotAnalyzer.calculate_safety_dist()
        a_time_over_pos = []
        for i in range(len(a_v_profile)):
            if i < a_start // 10:
                a_time_over_pos.append(-float('inf'))
            elif i == a_start // 10:
                a_time_over_pos.append(0)
            else:
                a_time_over_pos.append(a_time_over_pos[-1] + (10 / a_v_profile[i]))

        b_time_over_pos = []
        for i in range(len(b_v_profile)):
            if i < b_start // 10:
                b_time_over_pos.append(-float('inf'))
            elif i == b_start // 10:
                b_time_over_pos.append(0)
            else:
                b_time_over_pos.append(b_time_over_pos[-1] + (10 / b_v_profile[i]))

        contact = []
        point = -1
        for i in range(len(a_time_over_pos) - dist):
            assert len(b_time_over_pos) > i + dist
            if a_time_over_pos[i] < 0 or b_time_over_pos[i + dist] < 0:
                contact.append(-float('inf'))
            else:
                contact.append(b_time_over_pos[i + dist] - a_time_over_pos[i])
                if pre and point == -1:
                    if 0 <= contact[i] < float('inf'):
                        point = i
                else:
                    if -float('inf') < contact[i] <= 0:
                        point = i
                        break

        contact = contact + [float('inf')] * dist

        if point >= 0:
            return {'contact': True,
                    'index_a': point,
                    'index_b': point + dist,
                    'time_to_contact': a_time_over_pos[point],
                    'time_diffs': contact}
        else:
            return {'contact': False}

    @staticmethod
    def calculate_overtaking(slope, truck_a, truck_b, step_size, first_contact):
        """Calculates the overtaking maneuvers over all possible starting points

        Note: Uses multiprocessor Pool to speed up calculation. Calculation can take several seconds

        Args:
             slope: The slope over the full road
             truck_a: Dict with all details about truck A
             truck_b: Dict with all details about truck B
             step_size: The step size to sample the possible starting positions
             first_contact: Dict with all details about the point where Truck A reaches Truck B

        Returns:
            pc_details: The overtaking details on every sampled starting position with passive cooperation
            ac_details: The overtaking details on every sampled starting position with active cooperation
            overtaking: A dict containing details to the possible overtaking scenarios
        """
        osa = OvertakingSpotAnalyzer

        results = []
        if first_contact['contact']:
            indexes = range(0, len(first_contact['time_diffs']) - 30, step_size)

            args = [(i, slope, truck_a, truck_b, first_contact) for i in indexes if first_contact['time_diffs'][i] >= 0]
            pool = mp.Pool(processes=6)

            results = pool.starmap(osa.calculate_overtaking_at_index, args)

            pool.close()
            pool.join()

        pc_details, ac_details, overtaking = osa.generate_overtaking_options(results, slope, truck_a, truck_b,
                                                                             first_contact)

        return pc_details, ac_details, overtaking

    @staticmethod
    def calculate_overtaking_at_index(i, slope, truck_a, truck_b, first_contact):
        """Calculated the possible overtaking maneuvers at a specified starting index

        Note: This method does not run in shared memory due to Process Pool

        Args:
            i: index at which the overtaking maneuvers should start
            slope: The slope over the full road
            truck_a: Dict with all details about truck A
            truck_b: Dict with all details about truck B
            first_contact: Dict with all details about the point where Truck A reaches Truck B

        Returns:
            A 2D List with the overtaking results for passive and active cooperation
        """
        road_length = len(slope) * 50
        osa = OvertakingSpotAnalyzer
        dist = osa.calculate_safety_dist()

        result = [[], []]
        if first_contact['time_diffs'][i] >= 0:

            a_v_pre, b_v_pre = osa.v_profiles_pre_overtaking(i, first_contact['time_diffs'][i], truck_a, truck_b)

            # Try overtaking with passive cooperation _pc
            a_v_over_pc = osa.v_profile_a_overtaking_pc(slope, i, a_v_pre[-1], truck_a)
            overtaking_end_pc = osa.calculate_contact(truck_b['v_profile'], (i + dist) * 10,
                                                      a_v_over_pc, i * 10, False)

            if overtaking_end_pc['contact']:
                a_v_pc = a_v_pre + a_v_over_pc[i:]
                a_v_pc = a_v_pc[:road_length // 10 - truck_a['start'] // 10]
                a_fuel_pc, a_time_pc = osa.calculate_fuel_and_time_of_profile(slope, truck_a, a_v_pc, a_v_pc[0])

                result[0] = [i * 10, a_time_pc, a_fuel_pc, 0, 0, overtaking_end_pc['time_to_contact'], a_v_pc, 0]

            # Try overtaking with active cooperation _ac
            a_v_over_ac, b_v_over_ac = osa.v_profiles_overtaking_ac(slope, i, a_v_pre[-1], b_v_pre[-1], truck_a,
                                                                    truck_b)
            overtaking_end_ac = osa.calculate_contact(b_v_over_ac, (i + dist) * 10,
                                                      a_v_over_ac, i * 10, False)

            if overtaking_end_ac['contact']:
                over_end = overtaking_end_ac['index_a']

                a_v_post_ac, b_v_post_ac = osa.v_profiles_post_coop(slope, over_end,
                                                                    a_v_over_ac[over_end],
                                                                    b_v_over_ac[over_end - dist],
                                                                    truck_a, truck_b)

                a_v_ac = a_v_pre + a_v_over_ac[i:over_end] + a_v_post_ac
                a_v_ac = a_v_ac[:road_length // 10 - truck_a['start'] // 10]  # slope 50m vs segment 10m
                b_v_ac = b_v_pre + b_v_over_ac[i + dist:over_end - dist] + b_v_post_ac
                b_v_ac = b_v_ac[:road_length // 10 - truck_b['start'] // 10]

                a_fuel_ac, a_time_ac = osa.calculate_fuel_and_time_of_profile(slope, truck_a, a_v_ac, a_v_ac[0])
                b_fuel_ac, b_time_ac = osa.calculate_fuel_and_time_of_profile(slope, truck_b, b_v_ac, b_v_ac[0])

                result[1] = [i * 10, a_time_ac, a_fuel_ac, b_time_ac, b_fuel_ac,
                             overtaking_end_ac['time_to_contact'], a_v_ac, b_v_ac]
        return result

    @staticmethod
    def generate_overtaking_options(results, slope, truck_a, truck_b, first_contact):
        """Generates the detail dicts for the different overtaking options

        Args:
            results: The overtaking details on over position with PC and AC
            slope: The slope over the full road
            truck_a: Dict with all details about truck A
            truck_b: Dict with all details about truck B
            first_contact: Dict with all details about the point where Truck A reaches Truck B

        Returns:
            pc_details: The overtaking details on every sampled starting position with passive cooperation
            ac_details: The overtaking details on every sampled starting position with active cooperation
            overtaking: A dict containing details to the possible overtaking scenarios
        """
        osa = OvertakingSpotAnalyzer

        a_profile_first = truck_a['v_profile'][truck_a['start'] // 10:]
        a_fuel_first, a_time_first = osa.calculate_fuel_and_time_of_profile(slope, truck_a,
                                                                            a_profile_first, a_profile_first[0])

        b_profile_first = truck_b['v_profile'][truck_b['start'] // 10:]
        b_fuel_first, b_time_first = osa.calculate_fuel_and_time_of_profile(slope, truck_b,
                                                                            b_profile_first, b_profile_first[0])

        if first_contact['contact']:

            first = {'x': first_contact['index_a'] * 10,
                     'a_time': a_time_first,
                     'b_time': b_time_first,
                     'a_fuel': a_fuel_first,
                     'b_fuel': b_fuel_first,
                     'duration': None,
                     'a_profile': a_profile_first,
                     'b_profile': b_profile_first}

            roll_dist = 5  # start slowing down before the safety gap is reached to meet with truck b at same speed
            # roll dist of 5 is only a very rough approximation but sufficient
            a_profile_none_till_contact = truck_a['v_profile'][
                                          truck_a['start'] // 10:first_contact['index_a'] - roll_dist]
            _, a_profile_none_after_contact = osa.get_truck_with_profile(slope, truck_b['vel'],
                                                                         truck_a['weight'],
                                                                         min(truck_a['vel_p'], truck_a['vel_p']),
                                                                         truck_a['vel_m'],
                                                                         start_i=first_contact['index_a'] - roll_dist,
                                                                         start_v=a_profile_none_till_contact[-1])
            a_profile_none = a_profile_none_till_contact + a_profile_none_after_contact
            a_profile_none = a_profile_none[: len(slope) * 5]
            a_fuel_none, a_time_none = osa.calculate_fuel_and_time_of_profile(slope, truck_a, a_profile_none,
                                                                              a_profile_none[1])
            none = {'x': first_contact['index_a'] * 10,
                    'a_time': a_time_none,
                    'b_time': b_time_first,
                    'a_fuel': a_fuel_none,
                    'b_fuel': b_fuel_first,
                    'duration': None,
                    'a_profile': a_profile_none,
                    'b_profile': b_profile_first}

        else:
            first = {'x': None, 'a_time': None, 'b_time': None, 'a_fuel': None, 'b_fuel': None,
                     'duration': None, 'a_profile': None, 'b_profile': None}

            none = {'x': None,
                    'a_time': a_time_first,
                    'b_time': b_time_first,
                    'a_fuel': a_fuel_first,
                    'b_fuel': b_fuel_first,
                    'duration': None,
                    'a_profile': a_profile_first,
                    'b_profile': b_profile_first}

        stvo = {'x': None, 'a_time': None, 'b_time': None, 'a_fuel': None, 'b_fuel': None,
                'duration': None, 'a_profile': None, 'b_profile': None}

        pc_details = {'xs': [], 'a_times': [], 'b_times': [], 'a_fuels': [], 'b_fuels': [],
                      'durations': [], 'a_profiles': [], 'b_profiles': []}

        ac_details = {'xs': [], 'a_times': [], 'b_times': [], 'a_fuels': [], 'b_fuels': [],
                      'durations': [], 'a_profiles': [], 'b_profiles': []}

        if results:

            for result in results:
                if result[0]:
                    pc_details['xs'].append(result[0][0])
                    pc_details['a_times'].append(result[0][1])
                    pc_details['a_fuels'].append(result[0][2])
                    pc_details['b_times'].append(b_time_first)
                    pc_details['b_fuels'].append(b_fuel_first)
                    pc_details['durations'].append(result[0][5])
                    pc_details['a_profiles'].append(result[0][6])
                    pc_details['b_profiles'].append(b_profile_first)
                if result[1]:
                    ac_details['xs'].append(result[1][0])
                    ac_details['a_times'].append(result[1][1])
                    ac_details['a_fuels'].append(result[1][2])
                    ac_details['b_times'].append(result[1][3])
                    ac_details['b_fuels'].append(result[1][4])
                    ac_details['durations'].append(result[1][5])
                    ac_details['a_profiles'].append(result[1][6])
                    ac_details['b_profiles'].append(result[1][7])

            if pc_details['xs']:
                first['duration'] = pc_details['durations'][0]
                for (i, duration) in enumerate(pc_details['durations']):
                    if duration <= 45:  # v diff 11 km/h not 45 secs
                        stvo = {'x': pc_details['xs'][i],
                                'a_time': pc_details['a_times'][i],
                                'b_time': pc_details['b_times'][i],
                                'a_fuel': pc_details['a_fuels'][i],
                                'b_fuel': pc_details['b_fuels'][i],
                                'duration': pc_details['durations'][i],
                                'a_profile': pc_details['a_profiles'][i],
                                'b_profile': pc_details['b_profiles'][i]}
                        break

        overtaking = {'none': none, 'first': first, 'stvo': stvo}
        return pc_details, ac_details, overtaking

    @staticmethod
    def calculate_fuel_and_time_of_profile(slope, truck, v_profile, vel):
        """Calculated Time and Fuel required to drive a specified velocity profile

        Args:
            slope: The slope over the full road
            truck: Details over the truck that should drive the velocity profile
            v_profile: The velocity the truck should follow
            vel: The staring velocity of the truck at the beginning of the velocity profile

        Returns:
              fuel: The fuel required by the truck in litre
              time: The Time required by the truck in seconds
        """
        fuel = 0
        duration = 0
        start = (truck['start'] // 10)
        for i in range(len(v_profile)):
            v = v_profile[i]
            req_acc = (v - vel) / (10 / v)
            req_acc = req_acc / 2.2
            # the vel profile is eg. 22 22 23 23 24 24 -> double or trippe values due to discretisation
            # so there are at least two steps to reach the velocity, therefore the acceleration can be halved
            req_power = truck['truck'].engine.calculate_required_power(truck['truck'], req_acc, v,
                                                                       slope[int(((i + start) * 10) // 50)])
            a, f = truck['truck'].engine.apply_power(truck['truck'], req_power, v, slope[int(((i + start) * 10) // 50)])
            duration += (10 / v)
            fuel += f * (10 / v)
            vel = v

        return fuel / 1000, duration

    @staticmethod
    def v_profiles_pre_overtaking(pos, contact_time, truck_a, truck_b):
        """slows Truck A down so that they meet at the desired position, cuts profile till A is behind B
        shortens Truck B profile so that it ends when Truck A is right behind B"""
        dist = OvertakingSpotAnalyzer.calculate_safety_dist()

        a_start_index = truck_a['start'] // 10
        b_start_index = truck_b['start'] // 10

        time_for_each_segment = contact_time / (pos - a_start_index)
        a_v_pre = [10 / ((10 / e) + time_for_each_segment) for e in truck_a['v_profile'][a_start_index:pos]]

        b_v_pre = truck_b['v_profile'][b_start_index:pos + dist]
        return a_v_pre, b_v_pre

    @staticmethod
    def v_profiles_overtaking_ac(slope, pos, a_v, b_v, truck_a, truck_b):
        """calculates the velocity profiles for Truck A and B during the cooperative overtaking
        i.e. Truck A tries to accelerate and Truck B tries to slow down"""

        dist = OvertakingSpotAnalyzer.calculate_safety_dist()
        _, a_v_over = OvertakingSpotAnalyzer.get_truck_with_profile(slope, truck_a['vel'] + truck_a['vel_p'],
                                                                    truck_a['weight'],
                                                                    start_i=pos, start_v=a_v)
        a_v_over = [0] * pos + a_v_over

        # Get Profile for B during overtaking with v_minus
        # Due to the slope in 50m segments the resulting b_over_v_profile can be 40 m longer than desired.
        # Therefore the padded profile must be truncated to get fitting length
        # This might introduce an inaccuracy which is, however, negligible in its impact

        _, b_v_over = OvertakingSpotAnalyzer.get_truck_with_profile(slope, truck_b['vel'] + truck_b['vel_m'],
                                                                    truck_b['weight'],
                                                                    start_i=pos + dist, start_v=b_v)
        b_v_over = [0] * (pos + dist) + b_v_over
        b_v_over = b_v_over[:len(slope) * 5]
        return a_v_over, b_v_over

    @staticmethod
    def v_profiles_post_coop(slope, pos, a_v, b_v, truck_a, truck_b):
        """Calculates the velocity profile after an overtaking maneuver that ends at a specified position

        Note:
            The Overtaker wants to slow down, while the overtaken wants to accelerate (in case of AC)"""
        dist = OvertakingSpotAnalyzer.calculate_safety_dist()
        _, a_v_post_coop = OvertakingSpotAnalyzer.get_truck_with_profile(slope, truck_a['vel'], truck_a['weight'],
                                                                         truck_a['vel_p'],
                                                                         truck_a['vel_m'], a_v, pos)

        _, b_v_post_coop = OvertakingSpotAnalyzer.get_truck_with_profile(slope, truck_b['vel'], truck_b['weight'],
                                                                         truck_b['vel_p'],
                                                                         truck_b['vel_m'], b_v, pos - dist)

        return a_v_post_coop, b_v_post_coop

    @staticmethod
    def v_profile_a_overtaking_pc(slope, pos, start_v, truck_a):
        """Calculates the velocity Profile of Truck A during PC overtaking

        Note: Truck A has to accelerate if it slowed previously down to reach Truck B at a later position
        """
        __, a_v_over = OvertakingSpotAnalyzer.get_truck_with_profile(slope, truck_a['vel'], truck_a['weight'],
                                                                     truck_a['vel_p'], truck_a['vel_m'],
                                                                     start_i=pos, start_v=start_v)
        a_v_over = [0] * pos + a_v_over
        return a_v_over

    @staticmethod
    def asses_and_select_overtaking_spots(pc_details, ac_details, overtaking, cost_settings):
        """Monetarily asses each of the simulated overtaking maneuver ans selects the ones with the lowest total costs

        Note: Heavily depends on the number of cars which are assumed to be influenced by the overtaking truck
        """
        car_number = 10
        if cost_settings['type'] == 'Kock':
            cost_param = {'vot_car': car_number * 4.66 / 3600,
                          'vot_truck': 31.1 / 3600,
                          'vof': 3.83352,
                          'v_car': cost_settings['v_car']}
        else:
            cost_param = {'vot_car': car_number * 4.66 / 3600,
                          'vot_truck': 22.752 / 3600,
                          'vof': 1.3,
                          'v_car': cost_settings['v_car']}

        for mode in ['none', 'first', 'stvo']:
            if overtaking[mode]['a_time']:
                if overtaking[mode]['duration']:
                    car_delay = OvertakingSpotAnalyzer.calculate_car_delay(cost_param['v_car'],
                                                                           overtaking[mode]['a_profile'],
                                                                           overtaking[mode]['x'],
                                                                           overtaking[mode]['duration'])
                else:
                    car_delay = 0

                overtaking[mode]['cost'] = {'c_a_time': overtaking[mode]['a_time'] * cost_param['vot_truck'],
                                            'c_a_fuel': overtaking[mode]['a_fuel'] * cost_param['vof'],
                                            'c_b_time': overtaking[mode]['b_time'] * cost_param['vot_truck'],
                                            'c_b_fuel': overtaking[mode]['b_fuel'] * cost_param['vof'],
                                            'c_car': car_delay * cost_param['vot_car']}

        for details in [pc_details, ac_details]:
            if details['xs']:
                details['costs'] = []
                details['t_costs'] = []
                for i in range(len(details['xs'])):
                    car_delay = OvertakingSpotAnalyzer.calculate_car_delay(cost_param['v_car'],
                                                                           details['a_profiles'][i],
                                                                           details['xs'][i],
                                                                           details['durations'][i])

                    cost = {'c_a_time': details['a_times'][i] * cost_param['vot_truck'],
                            'c_a_fuel': details['a_fuels'][i] * cost_param['vof'],
                            'c_b_time': details['b_times'][i] * cost_param['vot_truck'],
                            'c_b_fuel': details['b_fuels'][i] * cost_param['vof'],
                            'c_car': car_delay * cost_param['vot_car']}
                    details['costs'].append(cost)
                    details['t_costs'].append(sum(cost.values()))

        if ac_details['xs']:
            best_ac_index = ac_details['t_costs'].index(min(ac_details['t_costs']))
            overtaking['ac'] = {'x': ac_details['xs'][best_ac_index],
                                'a_time': ac_details['a_times'][best_ac_index],
                                'b_time': ac_details['b_times'][best_ac_index],
                                'a_fuel': ac_details['a_fuels'][best_ac_index],
                                'b_fuel': ac_details['b_fuels'][best_ac_index],
                                'duration': ac_details['durations'][best_ac_index],
                                'a_profile': ac_details['a_profiles'][best_ac_index],
                                'b_profile': ac_details['b_profiles'][best_ac_index],
                                'cost': ac_details['costs'][best_ac_index]}
        else:
            overtaking['ac'] = {'x': None}

        if pc_details['xs']:
            best_pc_index = pc_details['t_costs'].index(min(pc_details['t_costs']))
            overtaking['pc'] = {'x': pc_details['xs'][best_pc_index],
                                'a_time': pc_details['a_times'][best_pc_index],
                                'b_time': pc_details['b_times'][best_pc_index],
                                'a_fuel': pc_details['a_fuels'][best_pc_index],
                                'b_fuel': pc_details['b_fuels'][best_pc_index],
                                'duration': pc_details['durations'][best_pc_index],
                                'a_profile': pc_details['a_profiles'][best_pc_index],
                                'b_profile': pc_details['b_profiles'][best_pc_index],
                                'cost': pc_details['costs'][best_pc_index]}
        else:
            overtaking['pc'] = {'x': None}

    @staticmethod
    def calculate_car_delay(v_car, vel_profile_a, start, duration):
        """Calculated how many seconds a car is delayed due to the overtaling maneuver"""
        elapsed_time = 0
        v_min = 1000
        for v in vel_profile_a[start // 10:]:
            elapsed_time += + 10 / v
            v_min = min(v, v_min)
            if elapsed_time >= duration:
                break
        time_without_overtaking = (duration * v_min) / v_car
        delay = duration - time_without_overtaking

        return max(0, delay)

    @staticmethod
    def calculate_safety_dist():
        """Calculated the required distance (front to front) to obtain a specified safety gap"""
        safety_gap = 50
        vehicle_length = 18.75
        dist = safety_gap + vehicle_length
        dist = math.ceil(dist / 10)
        return dist


class OsaTrigger:

    @staticmethod
    def wait_for_gui_input(queue_to_osa, queue_to_gui):
        """Handles the communication between GUI and OsaAnalyzer

        Args:
            queue_to_osa (queue): Pipeline from gui to osa
            queue_to_gui (queue): Pipeline form osa to gui
        """
        while True:
            try:
                data_from_gui = queue_to_osa.get_nowait()

                if data_from_gui[0] == 'query':
                    truck_a, truck_b, slope, step_size, cost_parameter = data_from_gui[1:]

                    analysis_results = OvertakingSpotAnalyzer.run_analysis(slope, truck_a, truck_b,
                                                                           step_size, cost_parameter)
                    queue_to_gui.put(['response', analysis_results])

            except queue.Empty:
                pass
            time.sleep(0.1)
