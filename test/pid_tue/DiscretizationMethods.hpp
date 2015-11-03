#ifndef DISCRETIZATIONMETHODS_HPP
#define DISCRETIZATIONMETHODS_HPP

namespace DFILTERS {

/// Discrretization methods as enum for easier understanding
enum DiscretizationMethod {
	EulerBackward = 1,
	EulerForward  = 2,
	Tustin        = 3,
	TustinPrewarp = 4,
	ZOH           = 5,
	ZeroPoleMatch = 6
};

}

#endif
