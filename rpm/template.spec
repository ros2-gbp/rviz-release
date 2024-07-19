%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/rolling/.*$
%global __requires_exclude_from ^/opt/ros/rolling/.*$

Name:           ros-rolling-rviz-rendering
Version:        14.2.4
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS rviz_rendering package

License:        BSD
URL:            https://github.com/ros2/rviz/blob/ros2/README.md
Source0:        %{name}-%{version}.tar.gz

Requires:       eigen3-devel
Requires:       qt5-qtbase
Requires:       qt5-qtbase-devel
Requires:       qt5-qtbase-gui
Requires:       ros-rolling-ament-index-cpp
Requires:       ros-rolling-eigen3-cmake-module
Requires:       ros-rolling-resource-retriever
Requires:       ros-rolling-rviz-assimp-vendor
Requires:       ros-rolling-rviz-ogre-vendor
Requires:       ros-rolling-ros-workspace
BuildRequires:  eigen3-devel
BuildRequires:  qt5-qtbase-devel
BuildRequires:  ros-rolling-ament-cmake-ros
BuildRequires:  ros-rolling-ament-index-cpp
BuildRequires:  ros-rolling-eigen3-cmake-module
BuildRequires:  ros-rolling-resource-retriever
BuildRequires:  ros-rolling-rviz-assimp-vendor
BuildRequires:  ros-rolling-rviz-ogre-vendor
BuildRequires:  ros-rolling-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  ros-rolling-ament-cmake-gmock
BuildRequires:  ros-rolling-ament-cmake-gtest
BuildRequires:  ros-rolling-ament-lint-auto
BuildRequires:  ros-rolling-ament-lint-common
%endif

%description
Library which provides the 3D rendering functionality in rviz.

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/rolling/setup.sh" ]; then . "/opt/ros/rolling/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/rolling" \
    -DAMENT_PREFIX_PATH="/opt/ros/rolling" \
    -DCMAKE_PREFIX_PATH="/opt/ros/rolling" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/rolling/setup.sh" ]; then . "/opt/ros/rolling/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/rolling/setup.sh" ]; then . "/opt/ros/rolling/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/rolling

%changelog
* Fri Jul 19 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 14.2.4-1
- Autogenerated by Bloom

* Tue Jun 25 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 14.2.3-1
- Autogenerated by Bloom

* Mon Jun 17 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 14.2.2-1
- Autogenerated by Bloom

* Mon Apr 29 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 14.2.0-1
- Autogenerated by Bloom

* Tue Apr 16 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 14.1.0-1
- Autogenerated by Bloom

* Sun Apr 07 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 14.0.0-1
- Autogenerated by Bloom

* Wed Mar 27 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 13.4.2-1
- Autogenerated by Bloom

* Tue Mar 26 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 13.4.1-1
- Autogenerated by Bloom

* Sat Mar 09 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 13.4.0-2
- Autogenerated by Bloom

* Sat Mar 09 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 13.4.0-1
- Autogenerated by Bloom

* Wed Mar 06 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 13.3.1-2
- Autogenerated by Bloom

