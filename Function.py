import math
import random

class Function:
    """ initialValue is the value the edge starts out with.
    """
    def __init__(self, initial_value):
        self.value = initial_value

    def update(self, time):
        return self.value


class Sine(Function):
    """ The parameters phase, amplitude, and angularVelocity are arrays of the respective sine functions
    that make up the edge. offset is what determines the min/max value along with the amplitude.
    It shifts the sine wave up or down the y-axis.
    """
    def __init__(self, phase, amplitude, angular_velocity, offset):
        self.phase = phase if not isinstance(phase, int) else [phase]
        self.amplitude = amplitude if not isinstance(amplitude, int) else [amplitude]
        self.angular_velocity = angular_velocity if not isinstance(angular_velocity, int) else [angular_velocity]
        self.num = len(phase) if not isinstance(phase, int) else 1
        self.offset = offset if not isinstance(offset, int) else [offset]
        Function.__init__(self, self.update(0))

    def update(self, time):
        new_val = 0
        for i in range(self.num):
            new_val += self.amplitude[i] * math.sin(self.angular_velocity[i] * time + self.phase[i]) + self.offset[i]
        self.value = new_val
        return self.value


class Square(Function):
    """ max and min represent the maximum and minimum values for the square wave.
        period is used to determine how often the function switches from max to min and vice versa
        phase determines the starting point for the function. The function is at its maximum when
        the count % period is less than half the period, and at the minimum otherwise.
    """
    def __init__(self, maximum, minimum, period, initial_phase):
        self.maximum = maximum
        self.minimum = minimum
        self.period = period
        self.phase = initial_phase
        Function.__init__(self, self.update(0))

    def update(self, time):
        if (self.phase + time) % self.period > (self.period / 2) - 1:
            self.value = self.maximum
            return self.maximum
        else:
            self.value = self.minimum
            return self.minimum


class NegExp(Function):
    """A function that uses the boids position to determine the value, by e^-position_mag() of the boid
    """
    def __init__(self, boid, host):
        self.boid = boid
        self.host = host
        Function.__init__(self, self.update(0))

    def update(self, time):
        self.value = pow(math.e, -abs(self.host.position_mag()) - self.boid.position_mag())
        return self.value


class CutNegExp(Function):
    """
    Same as the NegExp class, but when value <= cutoff the value will be 0
    """

    def __init__(self, boid, host, cutoff):
        self.boid = boid
        self.cutoff = cutoff
        self.host = host
        Function.__init__(self, self.update(0))

    def update(self, time):
        value = pow(math.e, -abs(self.host.position_mag()) - self.boid.position_mag())
        if value > self.cutoff:
            self.value = value
        else:
            self.value = 0
        return self.value


class Log(Function):
    """
    Value = Ln(6.5 - boid.position_mag())
    """

    def __init__(self, boid, host):
        self.boid = boid
        self.host = host
        Function.__init__(self, self.update(0))

    def update(self, time):
        pos_diff = self.host.position_mag() - self.boid.position_mag()
        self.value = math.log(.01 + abs(pos_diff))
        return self.value


class Sqrt(Function):
    """
    value = 1.7 - sqrt(boid.pos_magnitude())
    """

    def __init__(self, boid):
        self.boid = boid
        Function.__init__(self, self.update(0))

    def update(self, time):
        self.value = math.sqrt(3.5) - math.sqrt(self.boid.position_mag())
        return self.value


class DiffSqrt(Function):
    """
    value = 1.7 - sqrt(abs(host.position_mag() - boid.position_mag())
    """

    def __init__(self, boid, host):
        self.boid = boid
        self.host = host
        Function.__init__(self, self.update(0))

    def update(self, time):
        pos_diff = self.host.position_mag() - self.boid.position_mag()
        self.value = math.sqrt(3.5) - math.sqrt(abs(pos_diff))
        return self.value


class CompExp(Function):
    def __init__(self, boid, host, period):
        self.boid = boid
        self.host = host
        self.period = period
        Function.__init__(self, self.update(0))

    def update(self, time):
        pos_diff = abs(self.host.position_mag() - self.boid.position_mag())
        self.value = 2 * pow(math.e, -pos_diff) * (pow(math.cos(time % self.period), 2) + 2 * math.cos(time % self.period) * math.sin(time % self.period) - pow(math.sin(time % self.period), 2))
        return self.value


class RandInt(Function):
    def __init__(self, upper, lower):
        self.upper = upper
        self.lower = lower
        Function.__init__(self, self.update(0))

    def update(self, time):
        self.value = random.randint(self.lower, self.upper)
        return self.value


class ActiveFunction(Function):
    def __init__(self):
        Function.__init__(self, 0)

    def toggle(self, on):
        if on:
            self.value = 1
        else:
            self.value = 0
        return self.value