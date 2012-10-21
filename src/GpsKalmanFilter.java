
public class GpsKalmanFilter {
	public GpsKalmanFilter(double noise, 
			int state_dimension_p,
			int observation_dimension_p) {
		  /* The state model has four dimensions:
	     x, y, x', y'
	     Each time step we can only observe position, not velocity, so the
	     observation vector has only two dimensions.
		   */
		  timestep = 0;
		  state_dimension = 4;
		  observation_dimension = 2;

		  state_transition = new Matrix(state_dimension,
						    state_dimension);
		  observation_model = new Matrix(observation_dimension,
						     state_dimension);
		  process_noise_covariance = new Matrix(state_dimension,
							    state_dimension);
		  observation_noise_covariance = new Matrix(observation_dimension,
								observation_dimension);

		  observation = new Matrix(observation_dimension, 1);

		  predicted_state = new Matrix(state_dimension, 1);
		  predicted_estimate_covariance = new Matrix(state_dimension,
								 state_dimension);
		  innovation = new Matrix(observation_dimension, 1);
		  innovation_covariance = new Matrix(observation_dimension,
							 observation_dimension);
		  inverse_innovation_covariance = new Matrix(observation_dimension,
								 observation_dimension);
		  optimal_gain = new Matrix(state_dimension,
						observation_dimension);
		  state_estimate = new Matrix(state_dimension, 1);
		  estimate_covariance = new Matrix(state_dimension,
						       state_dimension);

		  vertical_scratch = new Matrix(state_dimension,
						    observation_dimension);
		  small_square_scratch = new Matrix(observation_dimension,
							observation_dimension);
		  big_square_scratch = new Matrix(state_dimension,
						      state_dimension);
		  
		  /* Assuming the axes are rectilinear does not work well at the
		     poles, but it has the bonus that we don't need to convert between
		     lat/long and more rectangular coordinates. The slight inaccuracy
		     of our physics model is not too important.
		   */
		  double v2p = 0.001;
		  state_transition.set_identity_matrix();
		  set_seconds_per_timestep(f, 1.0);
			     
		  /* We observe (x, y) in each time step */
		  observation_model.setAll(
			     1.0, 0.0, 0.0, 0.0,
			     0.0, 1.0, 0.0, 0.0);

		  /* Noise in the world. */
		  double pos = 0.000001;
		  process_noise_covariance.setAll(
			     pos, 0.0, 0.0, 0.0,
			     0.0, pos, 0.0, 0.0,
			     0.0, 0.0, 1.0, 0.0,
			     0.0, 0.0, 0.0, 1.0);

		  /* Noise in our observation */
		  observation_noise_covariance.setAll(
			     pos * noise, 0.0,
			     0.0, pos * noise);

		  /* The start position is totally unknown, so give a high variance */
		  state_estimate.setAll(0.0, 0.0, 0.0, 0.0);
		  estimate_covariance.set_identity_matrix();
		  
		  double trillion = 1000.0 * 1000.0 * 1000.0 * 1000.0;
		  estimate_covariance.scale(trillion);
	}
	
	/* The position units are in thousandths of latitude and longitude.
   The velocity units are in thousandths of position units per second.

   So if there is one second per timestep, a velocity of 1 will change
   the lat or long by 1 after a million timesteps.

   Thus a typical position is hundreds of thousands of units.
   A typical velocity is maybe ten.
	 */
	private void set_seconds_per_timestep(double seconds_per_timestep) {
		/* unit_scaler accounts for the relation between position and
     velocity units */
		double unit_scaler = 0.001;
		state_transition.set(0, 2, unit_scaler * seconds_per_timestep);
		state_transition.set(1, 3, unit_scaler * seconds_per_timestep);
	}

	/* Refer to http://en.wikipedia.org/wiki/Kalman_filter for
	   mathematical details. The naming scheme is that variables get names
	   that make sense, and are commented with their analog in
	   the Wikipedia mathematical notation.
	   This Kalman filter implementation does not support controlled
	   input.
	   (Like knowing which way the steering wheel in a car is turned and
	   using that to inform the model.)
	   Vectors are handled as n-by-1 matrices.
	   TODO: comment on the dimension of the matrices */
	
	 /* k */
	int timestep;
	
	/* These parameters define the size of the matrices. */
	int state_dimension, observation_dimension;
	
	/* This group of matrices must be specified by the user. */
	/* F_k */
	private Matrix state_transition;
	/* H_k */
	private Matrix observation_model;
	/* Q_k */
	Matrix process_noise_covariance;
	
	/* R_k */
	private Matrix observation_noise_covariance;
	
	/* The observation is modified by the user before every time step. */
	/* z_k */
	private Matrix observation;
	
	/* This group of matrices are updated every time step by the filter. */
	/* x-hat_k|k-1 */
	private Matrix predicted_state;
	
	/* P_k|k-1 */
	private Matrix predicted_estimate_covariance;
	/* y-tilde_k */
	private Matrix innovation;
	/* S_k */
	private Matrix innovation_covariance;
	/* S_k^-1 */
	private Matrix inverse_innovation_covariance;
	/* K_k */
	private Matrix optimal_gain;
	/* x-hat_k|k */
	private Matrix state_estimate;
	/* P_k|k */
	private Matrix estimate_covariance;
	
	/* This group is used for meaningless intermediate calculations */
	private Matrix vertical_scratch;
	private Matrix small_square_scratch;
	private Matrix big_square_scratch;
}
