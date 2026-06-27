from djitellopy import Tello
import time


class DroneController:
    def __init__(self):
        print("Create Tello object")
        self.drone = Tello()

    def connect(self):
        print("Connect to Tello Drone")

        try:
            self.drone.connect()

            battery_level = self.drone.get_battery()
            print(f"Battery Level: {battery_level}%")

            return True

        except Exception as e:
            print("Connection Failed:", e)
            return False

    def start_stream(self):
        print("Start Camera Stream")
        self.drone.streamon()

    def stop_stream(self):
        print("Stop Camera Stream")
        self.drone.streamoff()

    def takeoff(self):
        print("Take Off")
        self.drone.takeoff()

    def land(self):
        print("Land")
        self.drone.land()

    def get_frame(self):
        return self.drone.get_frame_read().frame

    def send_command(self, left_right, forward_backward, up_down, yaw):
        self.drone.send_rc_control(
            left_right,
            forward_backward,
            up_down,
            yaw
        )


if __name__ == "__main__":

    controller = DroneController()

    if controller.connect():

        controller.start_stream()
        time.sleep(2)

        controller.takeoff()

        controller.send_command(0, 30, 0, 0)
        time.sleep(2)
        
        controller.send_command(0, 0, 0, 0)

        frame = controller.get_frame()
        print(frame.shape)

        time.sleep(2)

        controller.land()
        controller.stop_stream()