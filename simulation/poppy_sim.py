import time
import math
import sys
from threading import Thread
from asciimatics.screen import Screen
import matplotlib.pyplot as plt

done_with_demo = False

# Payload
# 
# Simulates the state of a Poppins Payload (TM)
class Payload:
    
    def __init__(self, x, y, v, d, t):
        self.x = x
        self.y = y
        self.v = v
        self.d = d
        self.t = t

    # Move the payload according to velocity and direction
    def move(self):
        self.x = self.x + self.v * math.cos(self.d * math.pi / 180)
        self.y = self.y + self.v * math.sin(self.d * math.pi / 180)

    # Turn the payload (1 for left, -1 for right)
    def turn(self, direction):
        self.d = self.d + direction * 20
        if self.d < 0:
            self.d = 360 + self.d
        elif self.d > 360:
            self.d = self.d - 360

    # Drive (replica of arduino steering logic)
    def drive(self, t_x, t_y, loop_interval):
        global done_with_demo
        while 1:
            if done_with_demo:
                break

            if t_x != self.x:
                optimal_dir = math.atan((t_y - self.y) / (t_x - self.x)) * 180 / math.pi
            else:
                optimal_dir = math.atan((t_y - self.y) / (.00001)) * 180 / math.pi


            if t_y - self.y < 0 and t_x - self.x < 0:
                optimal_dir = optimal_dir + 180
            elif t_y - self.y > 0 and t_x - self.x < 0:
                optimal_dir = optimal_dir + 180
            if optimal_dir < 0:
                optimal_dir = optimal_dir + 360

            # Update boundaries
            left_b = optimal_dir + self.t
            if left_b > 360:
                left_b = left_b - 360

            right_b = optimal_dir - self.t
            if right_b < 0:
                right_b = right_b + 360

            center_b = optimal_dir + 180
            if center_b > 360:
                center_b = center_b - 360

            # Determine whether to turn

            # Turn right?
            turn_left = False
            turn_right = False
            left_added = False
            right_added = False
            dir_added = False
            actual_dir = self.d
            if center_b < left_b:
                left_added = True
                left_b = left_b - 360
                if actual_dir > center_b:
                    dir_added = True
                    actual_dir = actual_dir - 360
                if actual_dir < center_b and actual_dir > left_b:
                    turn_right = True
                if left_added:
                    left_b = left_b + 360
                    left_added = False
                if dir_added:
                    actual_dir = actual_dir + 360
                    dir_added = False
            elif not turn_right:
                if actual_dir < center_b and actual_dir > left_b:
                    turn_right = True
            
            # Turn left?
            if (center_b > right_b):
                right_added = True
                right_b = right_b + 360
                if actual_dir < center_b:
                    dir_added = True
                    actual_dir = actual_dir + 360
                if actual_dir > center_b and actual_dir < right_b:
                    turn_left = True
                if right_added:
                    right_b = right_b - 360
                    right_added = False
                if dir_added:
                    actual_dir = actual_dir - 360
                    dir_added = False
            elif not turn_left:
                if actual_dir > center_b and actual_dir < right_b:
                    turn_left = True

            # Perform turn
            if turn_right:
                self.turn(-1)
            elif turn_left:
                self.turn(1)

            # Go forward, ignoring wind
            self.move();

            # Print information
            #print(left_b, right_b, optimal_dir, actual_dir)
            #print(self.x, self.y, self.d, t_x, t_y, turn_right, turn_left)
    
            # Delay
            time.sleep(loop_interval)

    # Get status
    def get_status(self):
        return [self.x, self.y, self.d]


# Perform graphical demo by showing drive function using ascii art
# Also output a graph showing some nice data
def demo(screen, t_x, t_y, threshold, update_rate):
    global done_with_demo
    s_x = 0
    s_y = 0
    #t_x = 50
    #t_y = 25
    poppins = Payload(s_x, s_y, .5, 0, threshold)

    t = Thread(target=poppins.drive, args=(t_x, t_y, update_rate))
    t.start()
    info = poppins.get_status()

    screen.print_at('X', t_x, t_y, colour=3, bg=4)
    screen.refresh()

    distances = []

    while abs(s_x - t_x) > 2 and abs(s_y - t_y) > 2:
        screen.print_at('.', round(info[0]), round(info[1]), colour=4, bg=10)
        info = poppins.get_status()

        distances.append(math.sqrt((info[0] - t_x) ** 2 + (info[1] - t_y) ** 2))

        screen.print_at('O', round(info[0]), round(info[1]), colour=1, bg=2)
        screen.refresh()

        ev = screen.get_key()
        if ev in (ord('Q'), ord('q')):
            done_with_demo = True
            break
        
        time.sleep(update_rate)

    plt.plot(distances)
    plt.ylabel("Distance (m)")
    plt.xlabel("Time (s)")
    plt.title("Distance vs. Time of Payload with " + str(threshold) + " Degree Threshold")
    plt.grid(True)
    plt.show()

    t.join()
    

def main(argv):
    Screen.wrapper(demo, arguments=(int(argv[0]), int(argv[1]), int(argv[2]), float(argv[3])))


if __name__ == "__main__":
    main(sys.argv[1:])

