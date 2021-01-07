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
    def checking_obstacle(self,sensing_info):
        # 첫번째 가는경로에 장애물이 있다면 to_middle 범위로 판단
        # 차 두께 2.5m? 
        # 장애물 2m
        # 두번째 가는경로에 있는데(to_middle에 있는데 ) 커브에 있을경우 (나중에)
        obstacle_to_middle = sensing_info.track_forward_obstacles[0]['to_middle']
        obstacle_distance = sensing_info.track_forward_obstacles[0]['dist']
        if obstacle_to_middle-2.5 < sensing_info.to_middle < obstacle_to_middle + 2.5 and 0<obstacle_distance < 50:
            return True

        return False
    
    def set_throttle_by_speed(self,speed):
        if speed <= 100:  
            return 0.9
        if speed > 100: 
            return 0.8
        if speed > 130:  
            return 0.7
        if speed > 150:  
            return 0.55
        if speed > 180:
            return 0.5



    def set_steering_by_speed(self,ref_angle, sensing_info):
        steer_factor = sensing_info.speed * 1.5
        if sensing_info.speed > 70:
            steer_factor = sensing_info.speed * 0.85
        elif sensing_info.speed > 110:
            steer_factor = sensing_info.speed * 0.7
        return (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001)


    def obstacle_avoid(self, sensing_info, set_steering):
        obstacle_list = sensing_info.track_forward_obstacles
        half_load_width = self.half_road_limit - 1.25
        forward_obstacles = []
        not_available_pos = []
        
        if sensing_info.speed < 100:
            k = 1.8
        elif sensing_info.speed < 120:
            k = 1.6
        elif sensing_info.speed < 140:
            k = 1.5
        else:
            k = 1.3

        for obstacle in obstacle_list:
            if obstacle['dist'] >= 0 and obstacle['dist'] < (sensing_info.speed * 0.5) and abs(obstacle['to_middle']) <= (half_load_width + 1.0):
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


            set_steering_stay = 0
            check_list = []
            stack = []
            for i in available_pos:
                if not stack:
                    stack.append(i)
                elif stack[-1] + 0.5 == i:
                    stack.append(i)
                else:
                    check_list.append(stack)
                    stack = [i]
            for i in check_list:
                min_target_x = i[0]
                max_target_x = i[-1]
                target_y =forward_obstacles[0]['dist']
                min_temp_steering = (math.atan(min_target_x - sensing_info.to_middle) / target_y) - math.radians(sensing_info.moving_angle)
                max_temp_steering = (math.atan(max_target_x - sensing_info.to_middle) / target_y) - math.radians(sensing_info.moving_angle)
                if min_temp_steering *k <= set_steering <= max_temp_steering*k:
                    set_steering_stay = 1
                    break
            


            target_x = available_pos[min_idx]
            target_y = forward_obstacles[0]['dist']

            temp_steering = (math.atan((target_x - sensing_info.to_middle) / target_y) - math.radians(
                sensing_info.moving_angle))

            if set_steering_stay:
                temp_steering = set_steering
            else:
                temp_steering *= k

            return temp_steering
        
        return set_steering

    def control_driving(self, car_controls, sensing_info):
        # 변수 초기화
        set_steering, set_throttle, set_brake = 0, 0, 0
        half_load_width = self.half_road_limit - 1.25

        angle_num = int(sensing_info.speed / 45)
        ref_angle = sensing_info.track_forward_angles[angle_num]
        middle_add = (sensing_info.to_middle / 80) * -1
        throttle_factor = 0.6 / (abs(ref_angle) + 0.1)

        set_throttle = self.set_throttle_by_speed(sensing_info.speed)
        set_steering = self.set_steering_by_speed(ref_angle, sensing_info)


        if sensing_info.track_forward_obstacles and self.checking_obstacle(sensing_info):
            set_steering = self.obstacle_avoid(sensing_info, set_steering)

            if abs(set_steering) > 0.5 and sensing_info.track_forward_obstacles[0]['dist'] < 10:
                set_brake = 0.7

            if sensing_info.speed < 100:
                set_steering *= 1.7
            elif sensing_info.speed < 120:
                set_steering *= 1.5
            elif sensing_info.speed >= 120:
                set_brake = 0.4
                set_steering *= 1.3
            elif sensing_info.speed > 150:
                set_brake = 0.8
                set_steering *= 1

        else:
            set_steering += middle_add * 0.7


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
        
        if throttle_factor > 0.11 and not sensing_info.track_forward_obstacles:
            set_throttle = 1



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
