source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build \
	--symlink-install \
	--continue-on-error \
	--cmake-args \
	-DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
	-DUSE_LIVOX=OFF \
	-DCMAKE_BUILD_TYPE=Release
