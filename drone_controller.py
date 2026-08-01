import time
from djitellopy import Tello


class DroneController:

    def __init__(self):
        print("Create Tello object")

        self.drone = Tello()

        # 드론 상태 관리
        self.is_flying = False
        self.is_streaming = False


    def connect(self):

        print("Connect to Tello Drone")

        try:

            self.drone.connect()

            battery_level = self.drone.get_battery()

            print(
                f"Battery Level: {battery_level}%"
            )

            return True


        except Exception as e:

            print(
                "Connection Failed:",
                e
            )

            return False



    def get_battery(self):

        """
        배터리 잔량 확인
        """

        return self.drone.get_battery()



    def disconnect(self):

        """
        드론 연결 종료
        """

        print("Disconnect Drone")

        self.drone.end()



    def start_stream(self):

        print("Start Camera Stream")

        self.drone.streamon()

        self.is_streaming = True



    def stop_stream(self):

        print("Stop Camera Stream")

        if self.is_streaming:

            self.drone.streamoff()

            self.is_streaming = False



    def takeoff(self):

        print("Take Off")

        self.drone.takeoff()

        self.is_flying = True



    def land(self):

        print("Land")

        if self.is_flying:

            self.drone.land()

            self.is_flying = False



    def rotate_360(self):

        print("Rotate 360 Degrees")

        self.drone.rotate_clockwise(360)



    def get_frame(self):

        return self.drone.get_frame_read().frame



    def send_command(
        self,
        left_right,
        forward_backward,
        up_down,
        yaw
    ):

        self.drone.send_rc_control(
            left_right,
            forward_backward,
            up_down,
            yaw
        )



# 테스트 코드
if __name__ == "__main__":


    controller = DroneController()


    if controller.connect():


        try:

            controller.start_stream()

            time.sleep(2)


            controller.takeoff()

            time.sleep(2)


            # 앞으로 이동

            controller.send_command(
                0,
                30,
                0,
                0
            )

            time.sleep(2)


            controller.send_command(
                0,
                0,
                0,
                0
            )


            time.sleep(1)


            # 360도 회전

            controller.rotate_360()

            time.sleep(2)



            # 뒤로 이동

            controller.send_command(
                0,
                -30,
                0,
                0
            )

            time.sleep(2)



            controller.send_command(
                0,
                0,
                0,
                0
            )


            frame = controller.get_frame()


            if frame is not None:

                print(
                    "Frame shape:",
                    frame.shape
                )


        except Exception as e:


            print(
                "Operation Error:",
                e
            )


        finally:


            controller.land()

            controller.stop_stream()

            controller.disconnect()