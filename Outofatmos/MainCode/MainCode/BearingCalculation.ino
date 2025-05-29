// --- Bearing Calculation ---
// float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
//   float dLon = radians(lon2 - lon1);
//   float y = sin(dLon) * cos(radians(lat2));
//   float x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
//   float brng = atan2(y, x);
//   brng = degrees(brng);
//   if (brng < 0) brng += 360;
//   return brng;
// }
