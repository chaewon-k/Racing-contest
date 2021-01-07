from DrivingInterface.drive_controller import DrivingController
import math


class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = False

        # api or keyboard
        self.enable_api_control = True  # True(Controlled by code) /False(Controlled by keyboard)
        super().set_enable_api_control(self.enable_api_control)

        self.track_type = 99

        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0
        self.accident_step = 0
        self.reverse_recovery_count = 0
        self.reverse_drive_count = 0
        self.reverse_drive = 0
        self.reverse_complete = 0
        self.reverse_steering = 0
        self.reverse_complete_cnt = 0

        self.penalty_count = 0
        self.penalty_recovery = 0
        #
        # Editing area ends
        # ==========================================================#
        super().__init__()

    def obstacle_avoid(self, sensing_info):
        obstacle_list = sensing_info.track_forward_obstacles
        half_load_width = self.half_road_limit - 1.25
        forward_obstacles = []
        not_available_pos = []
        for obstacle in obstacle_list:
            if obstacle['dist'] >= 0 and obstacle['dist'] < (sensing_info.speed * 0.5) and abs(
                    obstacle['to_middle']) <= (half_load_width + 1.0):
                forward_obstacles.append({'dist': obstacle['dist'], 'to_middle': obstacle['to_middle'], 'speed': 0.0})
                pos = math.floor((obstacle['to_middle'] - 2) * 2) / 2
                for i in range(9):
                    if pos > half_load_width: pos = half_load_width
                    if pos < -half_load_width: pos = -half_load_width
                    not_available_pos.append(pos)
                    pos += 0.5

        if len(forward_obstacles) > 0:
            available_pos = []

            pos = -half_load_width
            for i in range(int(half_load_width * 4) + 1):
                available_pos.append(pos)
                pos += 0.5
            not_available_pos = list(set(not_available_pos))
            for not_available in not_available_pos:
                available_pos.remove(not_available)
            diff_pos = []
            for pos in available_pos:
                diff_pos.append(abs(sensing_info.to_middle - pos))
            min_idx = diff_pos.index(min(diff_pos))
            target_x = available_pos[min_idx]
            target_y = forward_obstacles[0]['dist']

            temp_steering = (math.atan((target_x - sensing_info.to_middle) / target_y) - math.radians(
                sensing_info.moving_angle))
            left_side = []
            right_side = []
            check = 0
            last_dist = 0
            for i in available_pos:
                if last_dist + 0.5 < i:
                    check += 1
                if not check:
                    last_dist = i
                    angle = (math.atan((i - sensing_info.to_middle) / target_y) - math.radians(
                        sensing_info.moving_angle))
                    left_side.append(angle)
                else:
                    last_dist = i
                    angle = (math.atan((i - sensing_info.to_middle) / target_y) - math.radians(
                        sensing_info.moving_angle))
                    right_side.append(angle)

            #############################################

            if sensing_info.speed < 100:k = 1.9
            elif sensing_info.speed < 120: k = 1.7
            elif sensing_info.speed < 140: k = 1.6
            else: k = 1.4

            #############################################

            return [temp_steering * k, left_side, right_side]
        return 0

    def set_throttle_by_speed(self,sensing_info):
        if sensing_info.speed <= 100: return 0.9
        if sensing_info.speed > 100: return 0.8
        if sensing_info.speed > 130: return 0.7
        if sensing_info.speed > 150: return 0.55
        if sensing_info.speed > 180: return 0.5

    def set_steering_by_speed(self,sensing_info):
        angle_num = int(sensing_info.speed / 45)
        ref_angle = sensing_info.track_forward_angles[angle_num]
        throttle_factor = 0.6 / (abs(ref_angle) + 0.1)
        steer_factor = sensing_info.speed * 1.5
        if sensing_info.speed > 70:
            steer_factor = sensing_info.speed * 0.85
        elif sensing_info.speed > 110:
            steer_factor = sensing_info.speed * 0.7

        return (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001)


    def control_driving(self, car_controls, sensing_info):
        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        set_steering, set_throttle, set_brake = 0,1,0
        half_load_width = self.half_road_limit - 1.25
        angle_num = int(sensing_info.speed / 45)
        ref_angle = sensing_info.track_forward_angles[angle_num]
        throttle_factor = 0.6 / (abs(ref_angle) + 0.1)
        middle_add = (sensing_info.to_middle / 80) * -1
        avoid_check = 0


        if sensing_info.speed > 10:
            self.accident_step = 0
            self.recovery_count = 0
            self.accident_count = 0

        if sensing_info.lap_progress > 0.5 and self.accident_step == 0 and abs(sensing_info.speed) < 1.0:
            self.accident_count += 1

        if self.accident_count > 5:
            self.accident_step = 1

        if self.accident_step == 1 and self.recovery_count < 12:
            self.recovery_count += 1
            if sensing_info.moving_angle >= 0:
                set_steering = 1
            else:
                set_steering = -1
            set_throttle = -1
            set_brake = 0

            car_controls.steering = set_steering
            car_controls.throttle = set_throttle
            car_controls.brake = set_brake

            return car_controls

        if self.recovery_count >= 12:
            self.accident_step = 2
            self.recovery_count = 0
            self.accident_count = 0

        if self.accident_step == 2:
            set_steering = 0.0
            set_throttle = 1
            set_brake = 1
            if sensing_info.speed > -1:
                self.accident_step = 0
                set_throttle = 1
                set_brake = 0


        # 역주행
        if not sensing_info.moving_forward and sensing_info.lap_progress > 1 and sensing_info.speed > 15:
            self.reverse_drive = 1
        if self.reverse_drive:
            self.reverse_drive_count += 1
        if self.reverse_drive and self.reverse_drive_count < 20:
            if sensing_info.to_middle >= 0:
                set_steering = 1
            else:
                set_steering = -1
            self.reverse_steering = set_steering
            set_throttle = -1
            set_brake = 0
            car_controls.steering = set_steering
            car_controls.throttle = set_throttle
            car_controls.brake = set_brake
            return car_controls

        if self.reverse_drive_count == 20:
            self.reverse_drive_count = 0
            self.reverse_drive = 0
            self.reverse_complete = 1

        if self.reverse_complete and self.reverse_complete_cnt < 5:
            set_steering = -self.reverse_steering
            set_brake = 1
            set_throttle = 0
            car_controls.steering = set_steering
            car_controls.throttle = set_throttle
            car_controls.brake = set_brake
            self.reverse_complete_cnt += 1
            return car_controls

        if self.reverse_complete and 5 <= self.reverse_complete_cnt < 15:
            set_steering = -self.reverse_steering
            set_brake = 0
            set_throttle = 1
            car_controls.steering = set_steering
            car_controls.throttle = set_throttle
            car_controls.brake = set_brake
            self.reverse_complete_cnt += 1
            return car_controls

        if self.reverse_complete_cnt == 15:
            self.reverse_complete = 0
            self.reverse_complete_cnt = 0

        ###########################################################

        set_throttle = self.set_throttle_by_speed(sensing_info)
        set_steering = self.set_steering_by_speed(sensing_info)

        if sensing_info.speed < 100: angle_num += 4
        elif sensing_info.speed < 120: angle_num += 7
        elif sensing_info.speed < 140: angle_num += 10
        else: angle_num += 13
        
        
        if sensing_info.track_forward_obstacles:
            for i in sensing_info.track_forward_obstacles:
                if sensing_info.to_middle - 2 < i['to_middle'] < sensing_info.to_middle + 2 and i['dist'] < angle_num * 10:
                    avoid_check = 1
                    break
        else:
            set_steering += middle_add * 0.7

        if avoid_check and 0 < sensing_info.track_forward_obstacles[0]['dist'] < (
        angle_num) * 10 :
            avoid_steering = self.obstacle_avoid(sensing_info)

            if not avoid_steering:
                set_steering += middle_add
            else:
                if set_steering > 0:
                    check = set_steering
                    if avoid_steering[0] > 0:
                        set_steering = max(set_steering, avoid_steering[0])

                    else:
                        if avoid_steering[2] and math.radians(ref_angle + sensing_info.moving_angle) > avoid_steering[2][0] and len(avoid_steering[2]) >= 3:
                            set_steering += avoid_steering[0] * 0.7
                        else:
                            set_steering = avoid_steering[0]
                    # if int(sensing_info.track_forward_obstacles[0]['dist'] // 10) == 1:
                    for i in range(int(sensing_info.track_forward_obstacles[0]['dist'] // 10)):
                        if sensing_info.track_forward_angles[i] > 30:
                            set_steering = check
                            break
                else:
                    check = set_steering
                    if avoid_steering[0] < 0: set_steering = min(set_steering, avoid_steering[0])
                    else:

                        if avoid_steering[1] and avoid_steering[1][-1] > math.radians(ref_angle + sensing_info.moving_angle) and len(avoid_steering[1]) >= 3:
                            set_steering -= avoid_steering[0] * 0.7
                        else:  # 못가는상황
                            set_steering = avoid_steering[0]
                    # if int(sensing_info.track_forward_obstacles[0]['dist'] // 10) == 1:
                    for i in range(int(sensing_info.track_forward_obstacles[0]['dist'] // 10)):
                        if sensing_info.track_forward_angles[i] < -30:
                            set_steering = check
                            break
                    

                if abs(set_steering) > 0.5 and sensing_info.track_forward_obstacles[0]['dist'] < 10:
                    set_brake = 0.7
                if sensing_info.speed < 100:
                    set_steering *= 1.9
                elif sensing_info.speed < 120:
                    set_steering *= 1.7
                elif sensing_info.speed <= 150:
                    set_steering *= 1.5
                elif sensing_info.speed > 150:
                    set_steering *= 1.4                    

        
        if avoid_check and sensing_info.speed > 40:
            set_brake = 0.2


        ## 긴급 및 예외 상황 처리 ##
        full_throttle = True
        emergency_brake = False

        road_range = int(sensing_info.speed / 30)
        for i in range(0, road_range):
            fwd_angle = abs(sensing_info.track_forward_angles[i])
            if fwd_angle > 45:
                full_throttle = False
            if fwd_angle > 80:
                emergency_brake = True
                break

        if full_throttle == False:
            if sensing_info.speed > 100:
                set_brake = 0.3
            if sensing_info.speed > 120:
                set_throttle = 0.7
                set_brake = 0.7
            if sensing_info.speed > 130:
                set_throttle = 0.5
                set_brake = 1.0

        if emergency_brake:
            if set_steering > 0:
                set_steering += 0.4
                set_throttle = 0.8
            else:
                set_steering -= 0.4
                set_throttle = 0.8

        if set_steering > 1 and sensing_info.lap_progress > 0:
            set_steering = 1

        elif set_steering < -1 and sensing_info.lap_progress > 0:
            set_steering = -1

        if throttle_factor > 0.11 and not sensing_info.track_forward_obstacles:
            set_throttle = 1

        ######
        if abs(sensing_info.to_middle) > half_load_width + 1.3:

            self.penalty_count += 1
        else:
            self.penalty_count = 0

        if self.penalty_count > 5 and self.penalty_recovery < 5:
            if sensing_info.to_middle < 0:
                set_steering = 0.5
            else:
                set_steering = -0.5
            self.penalty_recovery += 1

        if self.penalty_recovery == 5:
            self.penalty_recovery = 0



        # Moving straight forward
        car_controls.steering = set_steering
        car_controls.throttle = set_throttle
        car_controls.brake = set_brake

        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}" \
                  .format(car_controls.steering, car_controls.throttle, car_controls.brake))

        return car_controls

    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = ""
        return player_name


if __name__ == '__main__':
    print("[MyCar] Start Bot!")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot!")

    exit(return_code)