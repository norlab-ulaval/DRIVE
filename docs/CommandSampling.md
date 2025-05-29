# Command Sampling Module

DRIVE samples its command from the abstract `CommandSamplingStrategy` class. So, you can implement any command distribution as long as you derive that abstract class. By default, there is the `RandomSampling` which is a uniform sampling strategy.
