const std = @import("std");
pub const Float = f32;
pub const num_filters = 2;
pub const pow = std.math.pow;

/// IMM Parameters
pub const mu = [_]Float{
    0.8,
    0.2
};
pub const mu_rows = 2;
pub const mu_cols = 1;

pub const transition_matrix = [_]Float{
    0.95, 0.05,
    0.05, 0.95
};
pub const transition_matrix_rows = 2;
pub const transition_matrix_cols = 2;

/// Constant Acceleration Model
/// Constant acceleration model is the largest model for the IMM filter.
/// Since the IMM filter dictates that the size of both model parameters
/// be the same this will mean that the constant velocity model will have to
/// mirror the size of the CA model at 6x6
pub fn ca_state_transisition_matrix(dt: Float) [36]Float{
    return [_]Float{
        1, dt, pow(Float, dt, 2)/2, 0, 0,               0,   // x
        0, 1,                dt, 0, 0,                  0,            // x'
        0, 0,                1, 0,  0,                  0,            // x"
        0, 0,                0, 1, dt, pow(Float, dt, 2)/2, // y
        0, 0,                0, 0,  1,                  dt,           // y'
        0, 0,                0, 0,  0,                   1            // y"
    };
}
pub const state_transition_rows = 6;
pub const state_transition_cols = 6;


// Standard white noise continuous acceleration model
pub fn ca_process_noise_matrix(dt: Float) [36]Float{
    return [_]Float{
        pow(Float, dt, 5) / 20, pow(Float, dt, 4) / 8, pow(Float, dt, 3) / 6, 0, 0, 0,
        pow(Float, dt, 4) /  8, pow(Float, dt, 3) / 3, pow(Float, dt, 2) / 2, 0, 0, 0,
        pow(Float, dt, 3) /  6, pow(Float, dt, 2) / 2,                              dt, 0, 0, 0,
        0, 0, 0, pow(Float, dt, 5) / 20, pow(Float, dt, 4) / 8, pow(Float, dt, 3) / 6,
        0, 0, 0, pow(Float, dt, 4) /  8, pow(Float, dt, 3) / 3, pow(Float, dt, 2) / 2,
        0, 0, 0, pow(Float, dt, 3) /  6, pow(Float, dt, 2) / 2,                              dt,
    };
}
pub const process_noise_rows = 6;
pub const process_noise_cols = 6;

// Observation matrix for planar state transition matrix.
// Since there is no direct method of velocity measurement our measurements will be only x and y
// from the bounding box
pub const ca_observation_matrix = [_]Float{
    1, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0
};
pub const observation_matrix_rows = 2;
pub const observation_matrix_cols = 6;

// Measurement noise has no real method of measurement since we are only measuring
// pixels moving across the screen. For this assume 3 pixels of jitter to start.
pub const ca_measurement_noise_matrix = [_]Float{
    3, 0,
    0, 3
};
pub const measurement_noise_rows = 2;
pub const measurement_noise_cols = 2;


/// Constant Velocity Model
pub fn cv_state_transition_matrix(dt: Float) [36]Float{
    return [_]Float{
        1, dt,  0, 0,  0,  0, // x
        0,  1,  0, 0,  0,  0, // x'
        0,  0,  0, 0,  0,  0, // x"
        0,  0,  0, 1, dt,  0, // y
        0,  0,  0, 0,  1,  0, // y'
        0,  0,  0, 0,  0,  0, // y"
    };
}

// Continuous white acceleration noise with unit spectral density, matching CA's convention.
pub fn cv_process_noise_matrix(dt: Float) [36]Float{
    return [_]Float{
        pow(Float, dt, 3) / 3, pow(Float, dt, 2) / 2, 0, 0, 0, 0,
        pow(Float, dt, 2) / 2, dt,                    0, 0, 0, 0,
        0,                    0,                     0, 0, 0, 0,
        0, 0, 0, pow(Float, dt, 3) / 3, pow(Float, dt, 2) / 2, 0,
        0, 0, 0, pow(Float, dt, 2) / 2, dt,                    0,
        0, 0, 0, 0,                    0,                     0,
    };
}

// Both models measure the same x/y positions with the same measurement noise.
pub const cv_observation_matrix = ca_observation_matrix;
pub const cv_measurement_noise_matrix = ca_measurement_noise_matrix;



/// Initial Covariance estimate
pub const init_cov = [36]Float{
    10, 0, 0,  0, 0, 0,
    0,  5, 0,  0, 0, 0,
    0,  0, 1,  0, 0, 0,
    0,  0, 0, 10, 0, 0,
    0,  0, 0,  0, 5, 0,
    0,  0, 0,  0, 0, 1,
};
