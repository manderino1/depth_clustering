
/*
This inline function was automatically generated using DecisionTreeToCpp Converter
It takes feature vector as single argument:
feature_vector[0] - x_dim
feature_vector[1] - y_dim
feature_vector[2] - z_dim
feature_vector[3] - points
feature_vector[4] - distance
feature_vector[5] - area
feature_vector[6] - volume

It returns index of predicted class:
0 - car
1 - others

Simply include this file to your project and use it
*/
#include <vector>

inline int cluster_decision_tree(const std::vector<double> & feature_vector) 
{
	if (feature_vector.at(5) <= 17.22) {
		if (feature_vector.at(2) <= 0.36) {
			if (feature_vector.at(2) <= 0.23) {
				if (feature_vector.at(5) <= 0.04) {
					if (feature_vector.at(2) <= 0.07) {
						return 1;
					}
					else {
						return 0;
					}
				}
				else {
					if (feature_vector.at(4) <= 6.46) {
						return 1;
					}
					else {
						return 1;
					}
				}
			}
			else {
				if (feature_vector.at(0) <= 1.83) {
					if (feature_vector.at(4) <= 37.6) {
						return 1;
					}
					else {
						return 0;
					}
				}
				else {
					if (feature_vector.at(4) <= 9.73) {
						return 1;
					}
					else {
						return 1;
					}
				}
			}
		}
		else {
			if (feature_vector.at(2) <= 1.35) {
				if (feature_vector.at(1) <= 0.49) {
					if (feature_vector.at(1) <= 0.37) {
						return 1;
					}
					else {
						return 1;
					}
				}
				else {
					if (feature_vector.at(4) <= 85.82) {
						return 0;
					}
					else {
						return 1;
					}
				}
			}
			else {
				if (feature_vector.at(2) <= 1.4) {
					if (feature_vector.at(1) <= 1.98) {
						return 1;
					}
					else {
						return 0;
					}
				}
				else {
					if (feature_vector.at(2) <= 1.43) {
						return 1;
					}
					else {
						return 1;
					}
				}
			}
		}
	}
	else {
		if (feature_vector.at(2) <= 1.53) {
			if (feature_vector.at(4) <= 62.94) {
				if (feature_vector.at(2) <= 0.64) {
					if (feature_vector.at(0) <= 2.68) {
						return 0;
					}
					else {
						return 1;
					}
				}
				else {
					if (feature_vector.at(5) <= 165.41) {
						return 0;
					}
					else {
						return 1;
					}
				}
			}
			else {
				if (feature_vector.at(2) <= 1.53) {
					if (feature_vector.at(4) <= 67.99) {
						return 1;
					}
					else {
						return 1;
					}
				}
				else {
					if (feature_vector.at(4) <= 78.94) {
						return 0;
					}
					else {
						return 1;
					}
				}
			}
		}
		else {
			if (feature_vector.at(2) <= 2.15) {
				if (feature_vector.at(2) <= 2.15) {
					if (feature_vector.at(3) <= 19.5) {
						return 1;
					}
					else {
						return 1;
					}
				}
				else {
					return 0;
				}
			}
			else {
				if (feature_vector.at(4) <= 11.47) {
					if (feature_vector.at(3) <= 6596.5) {
						return 0;
					}
					else {
						return 1;
					}
				}
				else {
					if (feature_vector.at(3) <= 11.5) {
						return 1;
					}
					else {
						return 1;
					}
				}
			}
		}
	}
}