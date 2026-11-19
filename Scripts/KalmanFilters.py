import numpy as np
import typing

def create_scalar_array(value):
    return np.array(value).reshape(1, 1)

def multivar_noise_1d(cov):
    """
    Takes in covariance matrix and returns a 1D array of noise values from a multivariate normal distribution.
    :param cov: Covariance matrix
    :return: Noise vector
    """
    zero = np.zeros(cov.shape[0])
    return np.random.multivariate_normal(zero, cov, size = 1).reshape(-1, 1)


class InitializationError(Exception):
    pass

class DiscreteKalmanFilter:
    """
    Discrete Kalman Filter. Works with both scalar and matrix inputs
    """

    # Update Equations
    gain_update = staticmethod(lambda P, H, R: P @ H.T @ np.linalg.pinv(H @ P @ H.T + R))
    state_update = staticmethod(lambda x, z, K, H: x + K @ (z - H @ x))
    covariance_update_simple = staticmethod(lambda K, H, P, I: (I - K @ H) @ P)
    covariance_update_joseph = staticmethod(lambda K, H, P, R, I: (I - K @ H) @ P @ (I - K @ H).T + K @ R @ K.T)

    # Prediction Equations
    covariance_prediction = staticmethod(lambda F, P, Q: F @ P @ F.T + Q)
    state_prediction = staticmethod(lambda F, x: F @ x)


    def __init__(self, PHI, Q, R, H):
        self.F = self.scalar_check(PHI)
        self.Q = self.scalar_check(Q)
        self.R = self.scalar_check(R)
        self.H = self.scalar_check(H)
        self.K = None

        self.I = np.eye(self.F.shape[0]) if self.F.shape[0] > 1 else create_scalar_array(1)

        self.x_posterior = None
        self.cov_posterior = None
        self.x_prior = None
        self.cov_prior = None

        self.scalar_shape = (1, 1)

    @property
    def gain(self):
        return self.K

    @staticmethod
    def scalar_check(value):
        """
        Checks if value is scalar. If it is returns a (1,1) matrix
        :param value: value to check
        :return: np.ndarray matrix
        """
        if not np.isscalar(value):
            return value
        else:
            return create_scalar_array(value)

    def initialize(self, x, P):
        """
        Initializes the filter with the initial state and covariance
        :param x: Initial state
        :param P: Initial Covariance
        :return:
        """
        if x is None or P is None:
            err = 'Filter has not been initialized properly. Following variables are not properly set: '
            if not x:
                err += 'updated_covariance '
            if not self.x_posterior:
                err += 'updated_state_estimate '
            raise InitializationError(err)

        state = self.scalar_check(x)
        cov = self.scalar_check(P)

        self.x_prior = self.state_prediction(self.F, state).reshape(-1, 1)
        self.cov_prior = self.covariance_prediction(self.F, cov, self.Q)

    def predict(self) -> tuple:
        """
        Predicts state and covariance estimates.
        :return: tuple of prior state and prior covariance estimates
        """
        state = self.x_posterior.copy()
        cov = self.cov_posterior.copy()

        self.x_prior = self.state_prediction(self.F, state).reshape(-1, 1)
        self.cov_prior = self.covariance_prediction(self.F, cov, self.Q)

        return self.x_prior.copy(), self.cov_prior.copy()


    def update(self, z = None):
        """
        Updates the state and covariance based off the measurement z
        :param z: Measurement
        :return: tuple of state and covariance
        """

        self.K = self.gain_update(self.cov_prior, self.H, self.R)
        self.cov_posterior = self.covariance_update_joseph(self.K, self.H, self.cov_prior, self.R, self.I)

        if z is None:
            self.x_posterior = self.x_prior.copy()
        else:
            z = self.scalar_check(z)
            self.x_posterior = self.state_update(self.x_prior, z, self.K, self.H)

        if self.x_posterior.shape == self.scalar_shape and self.cov_posterior.shape == self.scalar_shape:
            return self.x_posterior.item(), self.cov_posterior.item()

        return self.x_posterior.copy(), self.cov_posterior.copy()


class UnscentedKalmanFilter:

    def __init__(self, dimension: int, UT_f: typing.Callable, UT_f_args: tuple, UT_h: typing.Callable, UT_h_args: tuple, Q: np.ndarray, R: np.ndarray):
        self.a = 0.001
        self.b = 2
        self.k = 0
        self.n = dimension
        self.L = 2 * dimension + 1
        self.UT_f = UT_f
        self.f_args = UT_f_args
        self.UT_h = UT_h
        self.h_args = UT_h_args
        self.Q = Q
        self.R = R
        self.K = None



        self.l = (self.a**2) * (self.n + self.k) - self.n
        self.nl = self.l + self.n


        self.weight_s, self.weight_c = self.calculate_weights()

        self.cov_prior = np.zeros((self.n, self.n), dtype = float)
        self.x_prior = np.zeros((self.n, 1), dtype = float)

        self.cov_posterior = np.zeros_like(self.cov_prior)
        self.x_posterior = np.zeros_like(self.x_prior)

        self.sigma_prior = None
        self.sigma_posterior = None



    def predict(self):
        self.sigma_prior = self.UT_f(self.sigma_posterior, *self.f_args)
        self.x_prior = self.calculate_mean(self.sigma_prior)

        self.cov_prior = self.calculate_cross_variance(self.sigma_prior, self.x_prior) + self.Q

        return self.x_prior, self.cov_prior

    def update(self, z):
        z_sigma = self.UT_h(self.sigma_prior, *self.h_args).reshape((1, -1))
        z_bar = self.calculate_mean(z_sigma)

        z_cov = self.calculate_cross_variance(z_sigma, z_bar) + self.R
        xz_cross = self.calculate_cross_variance(self.sigma_prior, self.x_prior, z_sigma, z_bar)
        self.K = self.calculate_gain(xz_cross, z_cov)

        self.x_posterior = self.update_state(self.x_prior, self.K, z, z_bar)
        self.cov_posterior = self.update_covariance(self.cov_prior, self.K,z_cov)

        self.sigma_posterior = self.calculate_sigma_points(self.cov_posterior)

        return self.x_posterior, self.cov_posterior

    def initialize(self, x, P):
        self.x_posterior[:] = x.copy()
        self.cov_posterior[:, :] = P.copy()

        self.sigma_posterior = self.calculate_sigma_points(self.cov_posterior)

    def calculate_sigma_points(self, cov):
        sqrt_cov = np.linalg.cholesky(self.nl * cov)
        sigma_points = np.zeros((self.n, self.L), dtype = float)

        sigma_points[:, 0] = self.x_posterior.flatten()

        for i in range(self.n):
            sigma_points[:, i + 1] = sigma_points[:, 0] + sqrt_cov[:, i]
            sigma_points[:, i + 1 + self.n] = sigma_points[:, 0] - sqrt_cov[:, i]

        return sigma_points

    def calculate_weights(self):
        weight_s = np.zeros((self.L,), dtype = float)
        weight_c = np.zeros_like(weight_s)

        weight_s[0] = self.l / self.nl
        weight_c[0] = weight_s[0] + (1 - self.a ** 2 + self.b)

        for i in range(1, self.L):
            weight_s[i] = weight_c[i] = 1 / (2 * self.nl)

        return weight_s, weight_c

    def calculate_mean(self, y):

        if y.ndim == 1:
           _y = y.reshape(1, -1)
        else:
            _y = y

        weights = np.tile(self.weight_s, (y.shape[0], 1))

        return np.sum(weights * _y, axis = 1, keepdims = True)

    def calculate_cross_variance(self, x, x_bar, y = None, y_bar = None):

        if y is None:
            y = x
        if y_bar is None:
            y_bar = x_bar

        Pxy = np.zeros((x.shape[0], y.shape[0]), dtype = float)
        for i in range(self.L):
            x_var = x[:, i] - x_bar[:, 0]
            y_var = y[:, i] - y_bar[:, 0]

            if x_var.ndim == 1:
                x_var = x_var.reshape(-1, 1)

            if y_var.ndim == 1:
                y_var = y_var.reshape(-1, 1)

            Pxy[:, :] += self.weight_c[i] * np.outer(x_var, y_var)

        return Pxy

    def predict_state(self, X):
        return self.calculate_mean(X)

    def predict_covariance(self, X, x_posterior):
        return self.calculate_cross_variance(X, x_posterior)

    @staticmethod
    def update_state(x_prior, gain, z, z_bar):
        return x_prior + gain @ (z - z_bar)

    @staticmethod
    def update_covariance(cov_prior, gain, measurement_cov):
        return cov_prior - gain @ measurement_cov @ gain.T

    @staticmethod
    def calculate_gain(p1, p2):
        return p1 @ np.linalg.pinv(p2)


class ExtendedKalmanFilter:
    covariance_predict = staticmethod(lambda F, P, Q: F @ P @ F.T + Q)
    gain_update = staticmethod(lambda P, H, R: P @ H.T @ np.linalg.pinv(H @ P @ H.T + R))
    state_update = staticmethod(lambda x, K, z, h: x + K * (z - h))
    covariance_update = staticmethod(lambda K, H, P, R, I: (I - K @ H) @ P @ (I - K @ H).T + K @ R @ K.T)
    state_prediction = staticmethod(lambda F, x: F @ x)

    def __init__(self, Q: np.ndarray, R: np.ndarray, F: (np.ndarray | None) = None, H: (np.ndarray | None) = None,
                 f: (typing.Callable | None) = None, f_args: tuple = None, h: (typing.Callable | None) = None, h_args: tuple = None,
                 f_jacobian: (typing.Callable | None) = None, f_jac_args: tuple = None,  h_jacobian: (typing.Callable | None) = None, h_jac_args: tuple = None):

        self.h_jac_args = h_jac_args
        self.f_jac_args = f_jac_args
        self.h_jacobian = h_jacobian
        self.f_jacobian = f_jacobian
        self.h_args = h_args
        self.h = h
        self.f_args = f_args
        self.f = f
        self.F = F
        self.Q = Q
        self.R = R
        self.H = H
        self.K = None
        self.I = None

        self.nonlinear_h = False
        self.nonlinear_f = False

        if self.H is None:
            self.nonlinear_h = True

        if self.F is None:
            self.nonlinear_f = True


        self.x_posterior = None
        self.cov_posterior = None
        self.x_prior = None
        self.cov_prior = None

    @property
    def gain(self):
        return self.K

    def initialize(self, x, P):
        """
        Initializes the filter with the initial state and covariance
        :param x: Initial state
        :param P: Initial Covariance
        :return:
        """
        self.x_posterior = x
        self.cov_posterior = P

    def update(self, z):

        if self.nonlinear_h:
            self.H = self.h_jacobian(self.x_prior, *self.h_jac_args)
            _h = self.h(self.x_prior, *self.h_args)
        else:
            _h = self.H @ self.x_prior

        self.I = np.eye(len(z.flatten()))

        self.K = self.gain_update(self.cov_prior, self.H, self.R)
        self.x_posterior = self.state_update(self.x_prior, self.K, z, _h)
        self.cov_posterior = self.covariance_update(self.K, self.H, self.cov_prior, self.R, self.I)

        return self.x_posterior.copy(), self.cov_posterior.copy()

    def predict(self):

        if self.nonlinear_f:
            _f = self.f_jacobian(self.x_posterior, *self.f_jac_args)
            _x = self.f(self.x_posterior, *self.f_args)
        else:
            _f = self.F
            _x = self.state_prediction(_f, self.x_posterior)


        self.x_prior = _x
        self.cov_prior = self.covariance_predict(_f, self.cov_posterior, self.Q)


        return self.x_prior.copy(), self.cov_prior.copy()