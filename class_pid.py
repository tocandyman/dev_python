class PIDController:
    def __init__(self, kp, ki, kd, sample_time):
        """
        Создает новый объект PID-регулятора.

        :param kp: коэффициент пропорциональности.
        :param ki: коэффициент интегральной составляющей.
        :param kd: коэффициент дифференциальной составляющей.
        :param sample_time: интервал времени между вычислениями управляющего воздействия.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sample_time = sample_time
        self.error = 0
        self.last_error = 0
        self.integral = 0

    def compute(self, setpoint, pv):
        """
        Вычисляет управляющее воздействие PID-регулятора.

        :param setpoint: заданное значение процесса.
        :param pv: текущее значение процесса.
        :return: значение управляющего воздействия.
        """
        self.error = setpoint - pv
        self.integral += self.error * self.sample_time
        derivative = (self.error - self.last_error) / self.sample_time
        output = self.kp * self.error + self.ki * self.integral + self.kd * derivative
        self.last_error = self.error
        return output
