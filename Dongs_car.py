from DrivingInterface.drive_controller import DrivingController
import numpy as np



class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #
        self.is_debug = False
        # api or keyboard
        self.enable_api_control = True # True(Controlled by code) /False(Controlled by keyboard)
        super().set_enable_api_control(self.enable_api_control)
        #
        # Editing area ends
        # ==========================================================#
        super().__init__()

    # def obstacle_avoid(self, track_forward_obstacles,sensing_info):
    #     half_load_width = self.half_road_limit - 1.25
    #     forward_obstacles = []
    #     not_available_pos = []
    #     for obstacle in track_forward_obstacles:
    #         if obstacle['dist'] >= 0 and obstacle['dist'] < (sensing_info.speed*0.5) and abs(obstacle['to_middle'])<=(half_load_width+1.0):
    #             forward_obstacles.append({'dist':obstacle['dist'],'to_middle':obstacle['to_middle'],'speed':0.0})
    #             pos = np.floor((obstacle['to_middle']-2)*2)/2
    #             for i in range(9):
    #                 if pos>half_load_width: pos=half_load_width
    #                 if pos < -half_load_width: pos=-half_load_width
    #                 not_available_pos.append(pos)
    #                 pos+=0.5


    #     if len(forward_obstacles) > 0:
    #         # forward_obstacles.sorted(key=lambda x:x[0])
    #         available_pos = []
    #         pos = -half_load_width   
    #         for i in range(int(half_load_width*4)+1):
    #             available_pos.append(pos)
    #             pos+=0.5
    #         not_available_pos = list(set(not_available_pos))
    #         for not_available in not_available_pos:
    #             available_pos.remove(not_available)
    #         diff_pos = []
    #         for pos in available_pos:
    #             diff_pos.append(abs(sensing_info.to_middle - pos))
    #         min_idx = diff_pos.index(min(diff_pos))
    #         target_x = available_pos[min_idx]
    #         target_y = forward_obstacles[0]['dist']

    #         set_steering =  (np.arctan((target_x-sensing_info.to_middle)/target_y)-np.radians(sensing_info.moving_angle))
    
    #         return set_steering




    
    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #
        if self.is_debug:
            print("=========================================================")
            print("[MyCar] to middle: {}".format(sensing_info.to_middle))

            print("[MyCar] collided: {}".format(sensing_info.collided))
            print("[MyCar] car speed: {} km/h".format(sensing_info.speed))

            print("[MyCar] is moving forward: {}".format(sensing_info.moving_forward))
            print("[MyCar] moving angle: {}".format(sensing_info.moving_angle))
            print("[MyCar] lap_progress: {}".format(sensing_info.lap_progress))

            print("[MyCar] track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("[MyCar] track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            print("[MyCar] opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            print("[MyCar] distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        ###########################################################################

        # Moving straight forward
        car_controls.steering = 0
        car_controls.throttle = 1
        car_controls.brake = 0
        
        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}"\
                .format(car_controls.steering, car_controls.throttle, car_controls.brake))

        #
        # Editing area ends
        # ==========================================================#
        
    
        ## 도로의 실제 폭의 1/2 로 계산됨
        half_load_width = self.half_road_limit - 1.2
        ## 차량의 차선 중앙 정렬을 위한 미세 조정 값 계산
        if sensing_info.speed < 120:
            middle_add = (sensing_info.to_middle / 80) * -1
        if sensing_info.speed > 120:
            middle_add = (sensing_info.to_middle / 100) * -1
        if sensing_info.speed > 160:
            middle_add = (sensing_info.to_middle / 120) * -1

        ## 차량 핸들 조정을 위해 참고할 전방의 커브 값 가져오기

        angle_num = int(sensing_info.speed / 45)

        ref_angle = sensing_info.track_forward_angles[angle_num]

        if sensing_info.speed < 120: set_throttle = 1  
        if sensing_info.speed > 120: set_throttle = 0.95
        if sensing_info.speed > 160: set_throttle = 0.75

        ## 전방의 커브 각도에 따라 throttle 값을 조절하여 속도를 제어함
        throttle_factor = 0.6 / (abs(ref_angle) + 0.1)
        # 전방이 직선에 가까울 경우

        if throttle_factor > 0.11: 
            throttle_factor = 0.11  
        set_throttle = 0.89 + throttle_factor
        


        steer_factor = sensing_info.speed * 1.5

        
        if sensing_info.speed > 70: 
            steer_factor = sensing_info.speed * 0.9        
        if sensing_info.speed > 100: 
            steer_factor = sensing_info.speed * 0.7
        if sensing_info.speed > 140: 
            steer_factor = sensing_info.speed * 0.6
        if sensing_info.speed > 160: 
            steer_factor = sensing_info.speed * 0.5


        set_steering = (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001)

        ## 차선 중앙정렬 값을 추가로 고려함
        set_steering += middle_add


        #장애물
        # a = 0
        # if len(sensing_info.track_forward_obstacles) > 0:
            
        #     a = self.obstacle_avoid(sensing_info.track_forward_obstacles,sensing_info)
        # if a:
        #     set_steering = a*1


        
            
            

        ## 긴급 및 예외 상황 처리 ########################################################################################
        full_throttle = True
        emergency_brake = False

        ## 전방 커브의 각도가 큰 경우 속도를 제어함
        ## 차량 핸들 조정을 위해 참고하는 커브 보다 조금 더 멀리 참고하여 미리 속도를 줄임
        road_range = int(sensing_info.speed / 30)
        for i in range(0, road_range):
            fwd_angle = abs(sensing_info.track_forward_angles[i])
            if 30<=fwd_angle <= 60:  ## 커브가 60도 이상인 경우 brake, throttle 을 제어
                full_throttle = True
            if fwd_angle > 60 and sensing_info.speed>160:  ## 커브가 80도 이상인 경우 steering 까지 추가로 제어
                emergency_brake = True
                break
            if fwd_angle > 80:  ## 커브가 80도 이상인 경우 steering 까지 추가로 제어
                emergency_brake = True
                break


        ## brake, throttle 제어
        set_brake = 0.0
        if full_throttle == False:
            if sensing_info.speed > 100:
                set_brake = 0.1
            if sensing_info.speed > 120:
                set_brake = 0.2
            if sensing_info.speed > 150:
                set_throttle = 1
                set_brake = 0.5
            if sensing_info.speed > 160:
                set_throttle = 1
                set_brake = 1


        ## steering 까지 추가로 제어
        if emergency_brake:
            print('emergency!')
            if set_steering > 0:
                set_steering += 0.3
                if sensing_info.speed > 160:
                    set_brake = 1
                    set_steering += 0.3
           
            else:
                set_steering -= 0.3
                if sensing_info.speed > 160:
                    set_brake = 1
                    set_steering -= 0.3
           


        car_controls.steering = set_steering
        car_controls.throttle = set_throttle
        car_controls.brake = set_brake
        

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
    print("[MyCar] Start Bot! (PYTHON)")

    client = DrivingClient()
    return_code = client.run()

    print("[MyCar] End Bot! (PYTHON)")

    exit(return_code)
