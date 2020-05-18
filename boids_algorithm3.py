import tkinter as tk
import random, cmath, math, copy
from cv2 import VideoWriter, VideoWriter_fourcc
from PIL import ImageGrab
import string

import numpy as np



class Window:
    def __init__(self):

        self.num_birds = 200 # Number of birds on screen
        self.birds = [Bird() for _ in range(self.num_birds)]
        # Walls or obstacles to keep away from
        self.walls = wall(1,1,1921,2,10) + wall(1,1,2,1081, 10) + wall(1,1080,1921,1081, 10) + wall(1920,1,1921,1081, 10)

        self.debug_mode = False

        # Display initializing
        self.root = tk.Tk()
        self.root.attributes("-fullscreen", True)
        self.root.bind("<Escape>", self.close)
        self.width, self.height = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        self.canvas = tk.Canvas(self.root, width=self.width, height=self.height)
        self.canvas.pack()

        if self.debug_mode:
            self.root.bind("<space>", self.debug_next)


        self.video_mode = False #
        if self.video_mode:
            self.fps = 30 # Number of frames per second in finished video
            self.video_length = self.fps * 1 # Number of frames in finished video
            self.framecount = 0 # Current frame

            # Video initializing
            letters = string.ascii_lowercase
            random_str = ''.join(random.choice(letters) for _ in range(4)) # Random id to prevent overwriting other videos
            path = "C:\\Koding\\Scripts\\Python\\Boids algorithm\\"
            file =  path + str(self.num_birds) + '_' + random_str + ".avi" # Finished video location
            fourcc = VideoWriter_fourcc(*'MP42')
            self.video = VideoWriter(file, fourcc, float(self.fps), (self.width, self.height))

            # Text to keep track of how long the video is.
            frame_text = str(self.framecount) + "/"+str(self.video_length)
            self.frame_text = self.canvas.create_text(70, 20, text=frame_text, font="Times 20 italic bold")

        self.root.after(400, self.frame)
        self.root.mainloop()

    def debug_next(self, event):
        self.frame()

    def save_frame(self):
        x=self.root.winfo_rootx()+self.canvas.winfo_x()
        y=self.root.winfo_rooty()+self.canvas.winfo_y()
        x1=x+self.canvas.winfo_width()
        y1=y+self.canvas.winfo_height()
        img = np.array(ImageGrab.grab().crop((x,y,x1,y1)))
        img = np.flip(img, axis=2)
        self.video.write(img)
        self.framecount += 1
        self.canvas.itemconfig(self.frame_text, text=str(self.framecount)+"/"+str(self.video_length))
        if self.framecount >= self.video_length:
            self.finished = True
            self.video.release()
            self.close(0)

    def frame(self):

        _ = [bird.update(self.birds, self) for bird in self.birds]
        _ = [bird.draw(self.canvas) for bird in self.birds]

        if self.video_mode:
            self.save_frame()

        if not self.debug_mode:
            self.root.after(1, self.frame)

    def draw(self, obj):
        pos = copy.copy(obj.pos)
        pos += complex(960,540)
        if obj.id == -1:
            obj.id = self.canvas.create_line(pos.real-self.bird_size, pos.imag - self.bird_size, pos.real+self.bird_size, pos.imag+self.bird_size, width=self.bird_size, fill='black')
        else:
            self.canvas.coords(obj.id, pos.real-self.bird_size, pos.imag - self.bird_size, pos.real+self.bird_size, pos.imag+self.bird_size)


    def close(self, event):
        self.root.withdraw()
        quit()

class Bird:

    def __init__(self):
        self.id = -1

        #Bird settings
        self.size = 5 # Size of bird in pixels

        self.speed = 5 # Number of pixels moved per second
        self.vector = random_vector() # ranomized starting vector
        self.pos = random_pos() # Randomized starting position
        self.range = 100 # Range of sigth (to other birds)
        self.wall_distance = 70 # Range of sight to obstacles
        self.distance = 20 # Distance away from other birds

        # ********************************************************
        # Weight of different elements that compute the new vector
        # ********************************************************
        self.self_weight = 5
        self.align_weight = 15
        self.cohesion_weight = 1
        self.steer_bird = lambda x : int(30 * math.exp(-0.3 * abs(x)))
        self.steer_wall = lambda x : int(40 * math.exp(-0.03 * abs(x)))


        self.new_vector, self.new_pos = 0j, 0j

    def draw(self, canvas):
        self.pos = self.new_pos
        self.vector = self.new_vector
        x, y = self.pos.real, self.pos.imag
        if self.id == -1:
            self.id = canvas.create_line(x, y, x + self.vector.real * self.size, y + self.vector.imag * self.size, width=self.size)
        else:
            canvas.coords(self.id, x, y, x + self.vector.real * self.size, y + self.vector.imag * self.size)

    def alignment(self, close_birds):
        # Alignment vector (Average of all vectors for other birds nearby)
        return average([bird.vector for bird in close_birds])

    def cohesion(self, close_birds):
        # Cohesion vector (Vector from bird to average point of all birds in range)
        ch_avg_point = average([bird.pos for bird in close_birds]) # Average point
        relative_pos = (ch_avg_point - self.pos) if abs(ch_avg_point-self.pos) > 0 else self.vector # cohesion point relative to self
        return relative_pos / abs(relative_pos) # Vector from self to cohesion point

    def steer(self, close_birds, walls):
        # Steer vector (opposite vector to average point of birds and walls within self.distance, excluding self)

        steer_wall = [p for p in walls if abs(self.pos - p) <= self.wall_distance] # walls
        steer_bird = [bird.pos for bird in close_birds if abs(bird.pos - self.pos) <= self.distance] # List of birds within self.distance from self

        avg_point = average(steer_bird) # Average poin
        relative_pos = (avg_point - self.pos) if abs(avg_point-self.pos) > 0 else 0 # Relative position
        steer_vector_bird = -(relative_pos / abs(relative_pos)) # Vector away from relative position
        steer_weight_bird = self.steer_bird(relative_pos)

        avg_point = average(steer_wall) # Average point
        relative_pos = (avg_point - self.pos) if abs(avg_point-self.pos) > 0 else 0 # Relative position
        steer_vector_wall = -(relative_pos / abs(relative_pos)) # Vector away from relative position
        steer_weight_wall = self.steer_wall(relative_pos)
        #steer_weight = min(2, int(1 / (abs(relative_pos) / 100))) # Steer weight is weighted higher the closer the bird is to other birds, max weight is 10

        return (steer_weight_bird, steer_vector_bird, steer_weight_wall, steer_vector_wall)




    def update(self, birds, window):

        close_birds = [] # List of birds within range, excluding self
        for bird in birds:
            if abs(bird.pos - self.pos) <= self.range and not self.id == bird.id:
                close_birds.append(bird)

        align_vector = self.alignment(close_birds)
        cohesion_vector = self.cohesion(close_birds)
        (steer_weight_bird, steer_vector_bird, steer_weight_wall, steer_vector_wall) = self.steer(close_birds, window.walls)

        new_vector = average(self.self_weight * [self.vector, ] \
                            + steer_weight_bird * [steer_vector_bird, ] \
                            + steer_weight_wall * [steer_vector_wall, ] \
                            + self.align_weight * [align_vector,] \
                            + self.cohesion_weight * [cohesion_vector,])

        self.new_vector = new_vector / abs(new_vector)
        self.new_pos = (self.new_vector * self.speed) + self.pos


# Tools:

def randomize(vector):
    x = vector.real + random.randint(-1000, 1000)/15000
    y = vector.imag + random.randint(-1000, 1000)/15000

    new_vector = complex(x,y)/abs(complex(x,y))
    return new_vector

def average(lst):
    # Average point on a complex plane from a list of points.
    if len(lst) > 0:
        return sum(lst)/len(lst)
    return 0j

def random_vector():
    # Random vector represented as a complex number with absolute value 1.
    return cmath.exp(1j*(random.randrange(0, int(2*cmath.pi * 100000))/100000))

def random_pos():
    # Random position on the plane represented by a complex number
    return complex(random.randrange(0, 1920), random.randrange(0, 1080))

def wall(x, y, x1, y1, a):
    # returns a list of points in a range, with step of 'a'
    xs = list(range(x, x1, a))
    ys = list(range(y, y1, a))
    xs = xs + [xs[0],] * (max(len(xs),len(ys)) - len(xs))
    ys = ys + [ys[0],] * (max(len(xs),len(ys)) - len(ys))
    return [complex(x_point, y_point) for x_point, y_point in zip(xs,ys)]

Window()
