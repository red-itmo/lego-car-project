import time


class Clock:
    """
        Clock for compute delta time
    """

    __start_time = 0
    __current_time = 0
    __last_time = 0
    __dt = 0

    def __init__(self):
        self.reset()

    def reset(self):
        self.__start_time = time.time()
        self.__current_time = 0
        self.__last_time = 0
        self.__dt = 0

    def update(self):
        self.__current_time = time.time() - self.__start_time
        self.__dt = self.__current_time - self.__last_time
        self.__last_time = self.__current_time

    def getDT(self):
        return self.__dt

    def getCurrentTime(self):
        return self.__current_time

    def getTandDT(self):
        self.update()
        return self.getCurrentTime(), self.getDT()


if __name__ == '__main__':
    clock = Clock()
    while True:
        # clock.update()
        t, dt = clock.getTandDT()
        print(t, dt)
        time.sleep(0.3)
