%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/jazzy/.*$
%global __requires_exclude_from ^/opt/ros/jazzy/.*$

Name:           ros-jazzy-rviz2
Version:        14.1.5
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS rviz2 package

License:        BSD
URL:            https://github.com/ros2/rviz/blob/ros2/README.md
Source0:        %{name}-%{version}.tar.gz

Requires:       python%{python3_pkgversion}-devel
Requires:       ros-jazzy-rviz-common
Requires:       ros-jazzy-rviz-default-plugins
Requires:       ros-jazzy-rviz-ogre-vendor
Requires:       ros-jazzy-ros-workspace
BuildRequires:  qt5-qtbase-devel
BuildRequires:  ros-jazzy-ament-cmake
BuildRequires:  ros-jazzy-rviz-common
BuildRequires:  ros-jazzy-rviz-ogre-vendor
BuildRequires:  ros-jazzy-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  python%{python3_pkgversion}-yaml
BuildRequires:  ros-jazzy-ament-cmake-cppcheck
BuildRequires:  ros-jazzy-ament-cmake-cpplint
BuildRequires:  ros-jazzy-ament-cmake-lint-cmake
BuildRequires:  ros-jazzy-ament-cmake-pytest
BuildRequires:  ros-jazzy-ament-cmake-uncrustify
BuildRequires:  ros-jazzy-ament-cmake-xmllint
BuildRequires:  ros-jazzy-ament-lint-auto
BuildRequires:  ros-jazzy-geometry-msgs
BuildRequires:  ros-jazzy-rclcpp
BuildRequires:  ros-jazzy-sensor-msgs
%endif

%description
3D visualization tool for ROS.

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/jazzy/setup.sh" ]; then . "/opt/ros/jazzy/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/jazzy" \
    -DAMENT_PREFIX_PATH="/opt/ros/jazzy" \
    -DCMAKE_PREFIX_PATH="/opt/ros/jazzy" \
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
if [ -f "/opt/ros/jazzy/setup.sh" ]; then . "/opt/ros/jazzy/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/jazzy/setup.sh" ]; then . "/opt/ros/jazzy/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/jazzy

%changelog
* Fri Sep 06 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 14.1.5-1
- Autogenerated by Bloom

* Mon Aug 19 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 14.1.4-1
- Autogenerated by Bloom

* Fri Jul 19 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 14.1.3-1
- Autogenerated by Bloom

* Thu Jun 27 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 14.1.2-1
- Autogenerated by Bloom

* Mon May 13 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 14.1.1-1
- Autogenerated by Bloom

* Fri Apr 19 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 14.1.0-2
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

