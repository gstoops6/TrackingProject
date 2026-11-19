import numpy as np
import scipy.stats as stat




class DiscreteKalmanFilter:
    """
    Discrete Kalman Filter. Works with both scalar and matrix inputs
    """

    # Update Equations
    gain_update = staticmethod(lambda P, H, R: P @ H.T @ np.linalg.pinv(H @ P @ H.T + R))
    state_update = staticmethod(lambda x, z, K, H: x + K @ (z - H @ x))
    covariance_update = staticmethod(lambda K, H, P, R, I: (I - K @ H) @ P @ (I - K @ H).T + K @ R @ K.T)

    # Prediction Equations
    covariance_prediction = staticmethod(lambda F, P, Q: F @ P @ F.T + Q)
    state_prediction = staticmethod(lambda F, x: F @ x)

    def __init__(self, n_dim, m_dim):

        self.x_dim = n_dim
        self.z_dim = m_dim

        self.F = np.zeros((n_dim, n_dim))
        self.Q = np.eye(n_dim)
        self.R = np.zeros((m_dim, m_dim))
        self.H = np.zeros((m_dim, n_dim))
        self.K = np.zeros((n_dim, m_dim))

        self.I = np.eye(n_dim)

        self._x_posterior = np.zeros((n_dim, 1))
        self._cov_posterior = np.eye(n_dim)
        self._x_prior = np.zeros_like(self._x_posterior)
        self._cov_prior = np.zeros_like(self._cov_posterior)

        self._log_likelihood = None
        self._likelihood = None

        self.x = self._x_posterior.copy()
        self.cov = self._cov_posterior.copy()
        self.z = np.zeros((m_dim, 1))

    def initialize(self, x, P):
        """
        Initializes the filter with the initial state and covariance
        :param x: Initial state
        :param P: Initial Covariance
        :return:
        """
        if x.shape != self._x_posterior.shape:
            if x.ndim == 1 and len(x) == self.x_dim:
                x = x.reshape(-1, 1)
            else:
                raise 'Incorrect Dimensions for state vector'
        if P.shape != self._cov_posterior.shape:
            raise 'Incorrect Dimensions for covariance matrix'

        self._x_posterior = self.state_prediction(self.F, x).reshape(-1, 1)
        self._cov_posterior = self.covariance_prediction(self.F, P, self.Q)
        self.x[:, :] = self._x_posterior[:, :]
        self.cov[:, :] = self._cov_posterior[:, :]


    def predict(self, x=None, P=None, F=None, Q=None) -> tuple:
        """
        Predicts state and covariance estimates.
        :return: tuple of prior state and prior covariance estimates
        """

        if F is not None:
            self.F = F

        if Q is not None:
            self.Q = Q

        if x is not None and P is not None:
            state = x
            cov = P
        else:
            state = self.x
            cov = self.cov

        self._x_prior[:, :] = self.state_prediction(self.F, state).reshape(-1, 1)
        self._cov_prior[:, :] = self.covariance_prediction(self.F, cov, self.Q)

        self.x[:, :] = self._x_prior[:, :]
        self.cov[:, :] = self._cov_prior[:, :]

        return self._x_prior.copy(), self._cov_prior.copy()

    def reinitialize(self, x, P):
        self.K[:, :] = 0
        self._x_posterior[:, :] = 0
        self._cov_posterior[:, :] = 0
        self._x_prior[:, :] = 0
        self._cov_prior[:, :] = 0
        self._log_likelihood = None
        self._likelihood = None

        self.initialize(x, P)
        self.predict()


    def update(self, z, H=None, R=None):
        """
        Updates the state and covariance based off the measurement z
        :return: tuple of state and covariance
        """
        self._likelihood = None
        self._log_likelihood = None

        if H is not None:
            self.H = H
        if R is not None:
            self.R = R

        if z.ndim == 1 and len(z) == self.z_dim:
            z = z.reshape(-1, 1)

        self.K[:, :] = self.gain_update(self._cov_prior, self.H, self.R)
        self._cov_posterior[:, :] = self.covariance_update(self.K, self.H, self._cov_prior, self.R, self.I)
        self._x_posterior[:, :] = self.state_update(self._x_prior, z, self.K, self.H)

        self.x[:, :] = self._x_posterior[:, :]
        self.cov[:, :] = self._cov_posterior[:, :]
        self.z[:, :] = z[:, :]

        return self._x_posterior.copy(), self._cov_posterior.copy()

    def update_state_model(self, update: dict | tuple):

        if isinstance(update, tuple) or isinstance(update, dict):
            for key, value in update:
                if isinstance(value, np.ndarray):
                    match key:
                        case 'F':
                            self.F[:, :] = value
                        case 'Q':
                            self.Q[:, :] = value
                        case 'R':
                            self.R[:, :] = value
                        case 'H':
                            self.H[:, :] = value
        else:
            raise ValueError(f'Update argument is not of type "dict" or "tuple". Type {type(update)}')

    @property
    def gain(self):
        return self.K

    @property
    def log_likelihood(self):
        if self._log_likelihood is None:
            y = self.z - self.H @ self._x_prior
            S = self.H @ self._cov_prior @ self.H.T + self.R
            self._log_likelihood = self._logpdf(x=y, cov=S)
        return self._log_likelihood

    @property
    def likelihood(self):
        if self._likelihood is None:
            self._likelihood = np.exp(self.log_likelihood)
            if self._likelihood == 0:
                self._likelihood = np.finfo(float).eps
        return self._likelihood

    @staticmethod
    def _logpdf(x, cov, allow_singular=True):
        flat_x = np.asarray(x).flatten()

        return stat.multivariate_normal.logpdf(x=flat_x, cov=cov, allow_singular=allow_singular)

class IMMFilter:

    def __init__(self, filters: tuple[DiscreteKalmanFilter] | list[DiscreteKalmanFilter],
                 mu: np.ndarray, M: np.ndarray, x_dim: int):

        self.filters = filters
        self.mu = mu / np.sum(mu, axis = 0)
        self.M = M



        self.N = len(filters)
        self._likelihood = np.zeros((self.N, 1))
        self._cbar = None
        self._omega = np.zeros((self.N, self.N))
        self._compute_mixing_probabilities()
        self._prediction_pending = False

        self._x_prior = np.zeros((x_dim, 1))
        self._cov_prior = np.zeros((x_dim, x_dim))
        self._x_posterior =  np.zeros((x_dim, 1))
        self._cov_posterior =  np.zeros((x_dim, x_dim))
        self.x =  np.zeros((x_dim, 1))
        self.cov =  np.zeros((x_dim, x_dim))


    def initialize(self, x, P):
        """
        Initializes the filter with the initial state and covariance
        :param x: Initial state
        :param P: Initial Covariance
        :return:
        """
        if x.shape != self._x_posterior.shape:
            print(x.shape, P.shape)
            raise ValueError(f'Initial state is not expected shape')

        for f in self.filters:
            f.initialize(x, P)
        self._prediction_pending = False

    def reinitialize(self, x, P):
        for f in self.filters:
            f.reinitialize(x, P)
        self._prediction_pending = False

    def update(self, z):
        for i, f in enumerate(self.filters):
            f.update(z)
            self._likelihood[i] = f.likelihood

        self.mu = self._cbar[:, :] * self._likelihood
        self.mu /= np.sum(self.mu, axis = 0)

        self._prediction_pending = False
        self._compute_mixing_probabilities()

        self._compute_state_estimate()
        self._x_posterior = self.x.copy()
        self._cov_posterior = self.cov.copy()

        return self._x_posterior.copy(), self._cov_posterior.copy()



    def predict(self):
        if self._prediction_pending:
            # The preceding prediction received no measurement update.
            self.mu = self._cbar.copy()
            self._compute_mixing_probabilities()

        xs, Ps = [], []
        for i, (f, w) in enumerate(zip(self.filters, self._omega.T)):
            x = np.zeros(self.x.shape)
            for kf, wj in zip(self.filters, w):
                x += getattr(kf, 'x') * wj
            xs.append(x)

            P = np.zeros(self.cov.shape)
            for kf, wj in zip(self.filters, w):
                y = getattr(kf, 'x')- x
                P += wj * (np.outer(y, y) + getattr(kf, 'cov'))
            Ps.append(P)

        #  compute each filter's prior using the mixed initial conditions
        for i, f in enumerate(self.filters):
            # propagate using the mixed state estimate and covariance
            setattr(f, 'x', np.array(xs[i].copy()).reshape(self.x.shape))
            setattr(f, 'cov', np.array(Ps[i].copy()).reshape(self.cov.shape))
            f.predict()

        # Combine predictions using the probabilities for the predicted time step.
        self._compute_state_estimate(self._cbar)
        self._x_prior = self.x.copy()
        self._cov_prior = self.cov.copy()
        self._prediction_pending = True



        return self._x_prior.copy(), self._cov_prior.copy()


    def _compute_mixing_probabilities(self):
        """
        Compute the mixing probability for each filter.
        """
        # M[i, j] is the probability of transitioning from model i to model j.
        self._cbar = self.M.T @ self.mu
        for i in range(self.N):
            for j in range(self.N):
                self._omega[i, j] = (self.M[i, j]*self.mu[i, 0]) / self._cbar[j, 0]

    def _compute_state_estimate(self, mode_probabilities=None):
        """
        Computes the IMM's mixed state estimate from each filter using
        the supplied mode probabilities, or self.mu for the posterior.
        """
        if mode_probabilities is None:
            mode_probabilities = self.mu

        self.x[:, :] = 0
        for f, mu in zip(self.filters, mode_probabilities.flatten()):
            _x = getattr(f, 'x')
            self.x += _x[:, :] * mu

        self.cov.fill(0)
        for f, mu in zip(self.filters, mode_probabilities.flatten()):
            _x = getattr(f, 'x')
            y = _x - self.x
            self.cov += mu * (np.outer(y, y) + getattr(f, 'cov'))

    def update_state_model(self, update: dict | tuple):

        if isinstance(update, tuple) or isinstance(update, dict):
            for f in self.filters:
                for key, value in update:
                    if isinstance(value, np.ndarray):
                        match key:
                            case 'F':
                                setattr(f, 'F', value)
                            case 'Q':
                                setattr(f, 'Q', value)
                            case 'R':
                                setattr(f, 'R', value)
                            case 'H':
                                setattr(f, 'H', value)
        else:
            raise ValueError(f'Update argument is not of type "dict" or "tuple". Type {type(update)}')

