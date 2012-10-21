GpsKalmanFilter
===============

Java port of ikalman, originally by Kevin Lacker

  https://github.com/mydrive/ikalman

Example
=======
GpsKalmanFilter f = new GpsKalmanFilter(1.0);
		
/* Move at a constant speed */
double lat = 0;
double long = 0;		
for (int i = 0; i < 20; ++i) {
	lat += 0.0001;
	long += 0.0001;
	f.updateVelocity(lat, long, 1.0 /*seconds since last update*/);

	// Get the smoothed lat/long
	double elat = f.getLat();
	double elon = f.getLong();

	// Get the smoothed lat/long speed per second
	double dlat = f.getVelocityLat();
	double dlon = f.getVelocityLong();
}
		

