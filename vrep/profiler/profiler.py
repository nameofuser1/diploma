import time


def timeit(fcn):
    def timed_fcn(*args, **kwargs):
        start = time.time()
        res = fcn(*args, **kwargs)
        end = time.time()

        ms = (end - start)*1000
        print(str(fcn) + " call has taken " + str(ms) + " milliseconds")
        return res

    return timed_fcn
