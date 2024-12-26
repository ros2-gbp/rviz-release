%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/humble/.*$
%global __requires_exclude_from ^/opt/ros/humble/.*$

Name:           ros-humble-rviz-rendering
Version:        11.2.15
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS rviz_rendering package

License:        BSD
URL:            https://github.com/ros2/rviz/blob/ros2/README.md
Source0:        %{name}-%{version}.tar.gz

Requires:       eigen3-devel
Requires:       qt5-qtbase
Requires:       qt5-qtbase-devel
Requires:       qt5-qtbase-gui
Requires:       ros-humble-ament-index-cpp
Requires:       ros-humble-eigen3-cmake-module
Requires:       ros-humble-resource-retriever
Requires:       ros-humble-rviz-assimp-vendor
Requires:       ros-humble-rviz-ogre-vendor
Requires:       ros-humble-ros-workspace
BuildRequires:  eigen3-devel
BuildRequires:  qt5-qtbase-devel
BuildRequires:  ros-humble-ament-cmake
BuildRequires:  ros-humble-ament-index-cpp
BuildRequires:  ros-humble-eigen3-cmake-module
BuildRequires:  ros-humble-resource-retriever
BuildRequires:  ros-humble-rviz-assimp-vendor
BuildRequires:  ros-humble-rviz-ogre-vendor
BuildRequires:  ros-humble-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  ros-humble-ament-cmake-cppcheck
BuildRequires:  ros-humble-ament-cmake-cpplint
BuildRequires:  ros-humble-ament-cmake-gmock
BuildRequires:  ros-humble-ament-cmake-gtest
BuildRequires:  ros-humble-ament-cmake-lint-cmake
BuildRequires:  ros-humble-ament-cmake-uncrustify
BuildRequires:  ros-humble-ament-cmake-xmllint
BuildRequires:  ros-humble-ament-lint-auto
%endif

%description
Library which provides the 3D rendering functionality in rviz.

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/humble" \
    -DAMENT_PREFIX_PATH="/opt/ros/humble" \
    -DCMAKE_PREFIX_PATH="/opt/ros/humble" \
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
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/humble

%changelog
* Thu Dec 26 2024 Jacob Perron <jacob@openrobotics.org> - 11.2.15-1
- Autogenerated by Bloom

* Mon Nov 25 2024 Jacob Perron <jacob@openrobotics.org> - 11.2.14-1
- Autogenerated by Bloom

* Mon Jul 29 2024 Jacob Perron <jacob@openrobotics.org> - 11.2.13-1
- Autogenerated by Bloom

* Tue Mar 26 2024 Jacob Perron <jacob@openrobotics.org> - 11.2.12-1
- Autogenerated by Bloom

* Fri Feb 16 2024 Jacob Perron <jacob@openrobotics.org> - 11.2.11-1
- Autogenerated by Bloom

* Wed Jan 24 2024 Jacob Perron <jacob@openrobotics.org> - 11.2.10-1
- Autogenerated by Bloom

* Mon Nov 13 2023 Jacob Perron <jacob@openrobotics.org> - 11.2.9-1
- Autogenerated by Bloom

* Tue Sep 19 2023 Jacob Perron <jacob@openrobotics.org> - 11.2.8-1
- Autogenerated by Bloom

* Thu Jul 27 2023 Jacob Perron <jacob@openrobotics.org> - 11.2.7-1
- Autogenerated by Bloom

* Tue Jul 18 2023 Jacob Perron <jacob@openrobotics.org> - 11.2.6-1
- Autogenerated by Bloom

* Tue Jan 10 2023 Jacob Perron <jacob@openrobotics.org> - 11.2.5-1
- Autogenerated by Bloom

* Mon Nov 07 2022 Jacob Perron <jacob@openrobotics.org> - 11.2.4-1
- Autogenerated by Bloom

* Mon Sep 12 2022 Jacob Perron <jacob@openrobotics.org> - 11.2.3-1
- Autogenerated by Bloom

* Wed May 11 2022 Jacob Perron <jacob@openrobotics.org> - 11.2.2-1
- Autogenerated by Bloom

* Tue Apr 26 2022 Jacob Perron <jacob@openrobotics.org> - 11.2.1-1
- Autogenerated by Bloom

* Tue Apr 19 2022 Jacob Perron <jacob@openrobotics.org> - 11.2.0-2
- Autogenerated by Bloom

* Fri Apr 08 2022 Jacob Perron <jacob@openrobotics.org> - 11.2.0-1
- Autogenerated by Bloom

* Wed Mar 30 2022 Jacob Perron <jacob@openrobotics.org> - 11.1.1-1
- Autogenerated by Bloom

* Thu Mar 24 2022 Jacob Perron <jacob@openrobotics.org> - 11.1.0-1
- Autogenerated by Bloom

* Tue Mar 01 2022 Jacob Perron <jacob@openrobotics.org> - 11.0.0-1
- Autogenerated by Bloom

* Wed Feb 16 2022 Jacob Perron <jacob@openrobotics.org> - 10.0.0-1
- Autogenerated by Bloom

* Tue Feb 15 2022 Jacob Perron <jacob@openrobotics.org> - 9.1.1-3
- Autogenerated by Bloom

* Tue Feb 08 2022 Jacob Perron <jacob@openrobotics.org> - 9.1.1-2
- Autogenerated by Bloom

