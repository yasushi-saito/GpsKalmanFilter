package com.ysaito.gpskalmanfilter;

public class GpsKalmanFilter {
	/** Create a GPS filter that only tracks two dimensions of position and
	   velocity.
	   The inherent assumption is that changes in velocity are randomly
	   distributed around 0.
	   
	   @param noise A parameter you can use to alter the expected noise.
	   1.0 is the original, and the higher it is, the more a path will be
	   "smoothed".
	   */
	public GpsKalmanFilter(double noise) {
		  /* The state model has four dimensions:
	     x, y, x', y'
	     Each time step we can only observe position, not velocity, so the
	     observation vector has only two dimensions.
		   */
		  timestep = 0;
		  mStateDimension = 4;
		  mObservationDimension = 2;

		  mStateTransition = new Matrix(mStateDimension,
						    mStateDimension);
		  mObservationModel = new Matrix(mObservationDimension,
						     mStateDimension);
		  mProcessNoiseCovariance = new Matrix(mStateDimension,
							    mStateDimension);
		  mObservationNoiseCovariance = new Matrix(mObservationDimension,
								mObservationDimension);

		  mObservation = new Matrix(mObservationDimension, 1);

		  mPredictedState = new Matrix(mStateDimension, 1);
		  mPredictedEstimateCovariance = new Matrix(mStateDimension,
								 mStateDimension);
		  mInnovation = new Matrix(mObservationDimension, 1);
		  mInnovationCovariance = new Matrix(mObservationDimension,
							 mObservationDimension);
		  mInverseInnovationCovariance = new Matrix(mObservationDimension,
								 mObservationDimension);
		  mOptimalGain = new Matrix(mStateDimension,
						mObservationDimension);
		  mStateEstimate = new Matrix(mStateDimension, 1);
		  mEstimateCovariance = new Matrix(mStateDimension,
						       mStateDimension);

		  mVerticalScratch = new Matrix(mStateDimension,
						    mObservationDimension);
		  mBigSquareScratch = new Matrix(mStateDimension,
						      mStateDimension);
		  
		  /* Assuming the axes are rectilinear does not work well at the
		     poles, but it has the bonus that we don't need to convert between
		     lat/long and more rectangular coordinates. The slight inaccuracy
		     of our physics model is not too important.
		   */
		  mStateTransition.setIdentityMatrix();
		  setSecondsPerTimestep(1.0);
			     
		  /* We observe (x, y) in each time step */
		  mObservationModel.setAll(
			     1.0, 0.0, 0.0, 0.0,
			     0.0, 1.0, 0.0, 0.0);

		  /* Noise in the world. */
		  double pos = 0.000001;
		  mProcessNoiseCovariance.setAll(
			     pos, 0.0, 0.0, 0.0,
			     0.0, pos, 0.0, 0.0,
			     0.0, 0.0, 1.0, 0.0,
			     0.0, 0.0, 0.0, 1.0);

		  /* Noise in our observation */
		  mObservationNoiseCovariance.setAll(
			     pos * noise, 0.0,
			     0.0, pos * noise);

		  /* The start position is totally unknown, so give a high variance */
		  mStateEstimate.setAll(0.0, 0.0, 0.0, 0.0);
		  mEstimateCovariance.setIdentityMatrix();
		  
		  double trillion = 1000.0 * 1000.0 * 1000.0 * 1000.0;
		  mEstimateCovariance.scale(trillion);
	}
	
	/** Set the seconds per timestep in the velocity2d model.
	 Defaults to one. 
	 */

	public void setSecondsPerTimestep(double seconds_per_timestep) {
	/*
	The position units are in thousandths of latitude and longitude.
   	The velocity units are in thousandths of position units per second.

   So if there is one second per timestep, a velocity of 1 will change
   the lat or long by 1 after a million timesteps.

   Thus a typical position is hundreds of thousands of units.
   A typical velocity is maybe ten.
	 */
		
		// unit_scaler accounts for the relation between position and
		// velocity units 
		final double unitScaler = 0.001;
		mStateTransition.set(0, 2, unitScaler * seconds_per_timestep);
		mStateTransition.set(1, 3, unitScaler * seconds_per_timestep);
	}

	/** Update the model with new gps data. 
	 * 
	 */
	public final void updateVelocity(double lat, double lon,
			double seconds_since_last_timestep) {
		setSecondsPerTimestep(seconds_since_last_timestep);
		mObservation.setAll(lat * 1000.0, lon * 1000.0);
		update();
	}

	/** Extract the current latitude from the kalman filter */
	public final double getLat() {
		return mStateEstimate.get(0, 0) / 1000.0;
	}
	
	/** Extract the current longitude from the kalman filter */
	public final double getLong() {
		return mStateEstimate.get(1, 0) / 1000.0;
	}
	
	/** Extract velocity with latitude-per-second units from the filter */
	public final double getVelocityLat() {
		return mStateEstimate.get(2, 0) / (1000.0 * 1000.0);
	}
	
	/** Extract velocity with longitude-per-second units from the filter */
	public final double getVelocityLong() {
		return mStateEstimate.get(3, 0) / (1000.0 * 1000.0);
	}
	
	/** Extract a bearing from a velocity2d Kalman filter.
	   0 = north, 90 = east, 180 = south, 270 = west */
	public double getBearing() {
		/* See
   http://www.movable-type.co.uk/scripts/latlong.html
   for formulas */
		double lat = getLat();
		double lon = getLong();
		double deltaLat = getVelocityLat();
		double deltaLon = getVelocityLong();
		double x, y;
		
		/* Convert to radians */
		double to_radians = Math.PI / 180.0;
		lat *= to_radians;
		lon *= to_radians;
		deltaLat *= to_radians;
		deltaLon *= to_radians;
		
		/* Do math */
		double lat1 = lat - deltaLat;
		y = Math.sin(deltaLon) * Math.cos(lat);
		x = Math.cos(lat1) * Math.sin(lat) - Math.sin(lat1) * Math.cos(lat) * Math.cos(deltaLon);
		double bearing = Math.atan2(y, x);

		/* Convert to degrees */
		bearing = bearing / to_radians;
		while (bearing >= 360.0) {
			bearing -= 360.0;
		}
		while (bearing < 0.0) {
			bearing += 360.0;
		}
		
		return bearing;
	}
	static final double EARTH_RADIUS_IN_MILES = 3963.1676;
	
	/** Convert a lat, long, delta lat, and delta long into mph.*/
	public static double calculateMph(double lat, double lon,
			double deltaLat, double deltaLon) {
		/* First, let's calculate a unit-independent measurement - the radii
     of the earth traveled in each second. (Presumably this will be
     a very small number.) */
  
		/* Convert to radians */
		double to_radians = Math.PI / 180.0;
		lat *= to_radians;
		lon *= to_radians;
		deltaLat *= to_radians;
		deltaLon *= to_radians;

		/* Haversine formula */
		double lat1 = lat - deltaLat;
		double sin_half_dlat = Math.sin(deltaLat / 2.0);
		double sin_half_dlon = Math.sin(deltaLon / 2.0);
		double a = sin_half_dlat * sin_half_dlat + Math.cos(lat1) * Math.cos(lat)
				* sin_half_dlon * sin_half_dlon;
		double radians_per_second = 2 * Math.atan2(1000.0 * Math.sqrt(a),
				1000.0 * Math.sqrt(1.0 - a));
		
		/* Convert units */
		double miles_per_second = radians_per_second * EARTH_RADIUS_IN_MILES;
		double miles_per_hour = miles_per_second * 60.0 * 60.0;
		return miles_per_hour;
	}
	
	/** Extract speed in miles per hour from a velocity2d Kalman filter. */
	public double getMph() {
		double lat = getLat();
		double lon = getLong();
		double delta_lat = getVelocityLat();
		double delta_lon = getVelocityLong();
		return calculateMph(lat, lon, delta_lat, delta_lon);
	}
	
	private final void update() {
		predict();
		estimate();
	}

	private void predict() {
		timestep++;

		/* Predict the state */
		Matrix.multiply(mStateTransition, mStateEstimate,
				mPredictedState);
		
		/* Predict the state estimate covariance */
		Matrix.multiply(mStateTransition, mEstimateCovariance,
				mBigSquareScratch);
		Matrix.multiplyByTransposeMatrix(mBigSquareScratch, mStateTransition,
				mPredictedEstimateCovariance);
		Matrix.add(mPredictedEstimateCovariance, mProcessNoiseCovariance,
				mPredictedEstimateCovariance);
	}

	void estimate() {
		/* Calculate innovation */
		Matrix.multiply(mObservationModel, mPredictedState,
				mInnovation);
		Matrix.subtract(mObservation, mInnovation,
				mInnovation);
		
		/* Calculate innovation covariance */
		Matrix.multiplyByTransposeMatrix(mPredictedEstimateCovariance,
				mObservationModel,
				mVerticalScratch);
		Matrix.multiply(mObservationModel, mVerticalScratch,
				mInnovationCovariance);
		Matrix.add(mInnovationCovariance, mObservationNoiseCovariance,
				mInnovationCovariance);
		
		/* Invert the innovation covariance.
		     Note: this destroys the innovation covariance.
		     TODO: handle inversion failure intelligently. */
		Matrix.invert(mInnovationCovariance,
				mInverseInnovationCovariance);
		
		/* Calculate the optimal Kalman gain.
		     Note we still have a useful partial product in vertical scratch
		     from the innovation covariance. */
		Matrix.multiply(mVerticalScratch, mInverseInnovationCovariance,
				mOptimalGain);
		
		/* Estimate the state */
		Matrix.multiply(mOptimalGain, mInnovation,
				mStateEstimate);
		Matrix.add(mStateEstimate, mPredictedState,
				mStateEstimate);
		
		/* Estimate the state covariance */
		Matrix.multiply(mOptimalGain, mObservationModel,
				mBigSquareScratch);
		mBigSquareScratch.subtractFromIdentityMatrix();
		Matrix.multiply(mBigSquareScratch, mPredictedEstimateCovariance,
				mEstimateCovariance);
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
	int mStateDimension, mObservationDimension;
	
	/* This group of matrices must be specified by the user. */
	/* F_k */
	private Matrix mStateTransition;
	/* H_k */
	private Matrix mObservationModel;
	/* Q_k */
	Matrix mProcessNoiseCovariance;
	
	/* R_k */
	private Matrix mObservationNoiseCovariance;
	
	/* The observation is modified by the user before every time step. */
	/* z_k */
	private Matrix mObservation;
	
	/* This group of matrices are updated every time step by the filter. */
	/* x-hat_k|k-1 */
	private Matrix mPredictedState;
	
	/* P_k|k-1 */
	private Matrix mPredictedEstimateCovariance;
	/* y-tilde_k */
	private Matrix mInnovation;
	/* S_k */
	private Matrix mInnovationCovariance;
	/* S_k^-1 */
	private Matrix mInverseInnovationCovariance;
	/* K_k */
	private Matrix mOptimalGain;
	/* x-hat_k|k */
	private Matrix mStateEstimate;
	/* P_k|k */
	private Matrix mEstimateCovariance;
	
	/* This group is used for meaningless intermediate calculations */
	private Matrix mVerticalScratch;
	private Matrix mBigSquareScratch;
}
