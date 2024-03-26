%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/iron/.*$
%global __requires_exclude_from ^/opt/ros/iron/.*$

Name:           ros-iron-rviz-ogre-vendor
Version:        12.4.7
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS rviz_ogre_vendor package

License:        Apache License 2.0 and MIT
URL:            https://www.ogre3d.org/
Source0:        %{name}-%{version}.tar.gz

Requires:       freetype
Requires:       freetype-devel
Requires:       libX11-devel
Requires:       libXaw-devel
Requires:       libXrandr-devel
Requires:       mesa-libGL-devel
Requires:       mesa-libGLU-devel
Requires:       ros-iron-ros-workspace
BuildRequires:  freetype-devel
BuildRequires:  git
BuildRequires:  libX11-devel
BuildRequires:  libXaw-devel
BuildRequires:  libXrandr-devel
BuildRequires:  mesa-libGL-devel
BuildRequires:  mesa-libGLU-devel
BuildRequires:  pkgconfig
BuildRequires:  ros-iron-ament-cmake
BuildRequires:  ros-iron-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  ros-iron-ament-cmake-xmllint
BuildRequires:  ros-iron-ament-lint-auto
%endif

%description
Wrapper around ogre3d, it provides a fixed CMake module and an ExternalProject
build of ogre.

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/iron" \
    -DAMENT_PREFIX_PATH="/opt/ros/iron" \
    -DCMAKE_PREFIX_PATH="/opt/ros/iron" \
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
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/iron

%changelog
* Tue Mar 26 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 12.4.7-1
- Autogenerated by Bloom

* Wed Feb 07 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 12.4.6-1
- Autogenerated by Bloom

* Fri Jan 19 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 12.4.5-2
- Autogenerated by Bloom

* Fri Nov 17 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 12.4.5-1
- Autogenerated by Bloom

* Fri Sep 08 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 12.4.4-1
- Autogenerated by Bloom

* Mon Aug 21 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 12.4.3-1
- Autogenerated by Bloom

* Thu Jul 27 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 12.4.2-1
- Autogenerated by Bloom

* Fri Jul 14 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 12.4.1-1
- Autogenerated by Bloom

* Thu Apr 20 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 12.4.0-2
- Autogenerated by Bloom

* Tue Apr 18 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 12.4.0-1
- Autogenerated by Bloom

* Tue Apr 11 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 12.3.2-1
- Autogenerated by Bloom

* Mon Mar 27 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 12.3.1-4
- Autogenerated by Bloom

* Mon Mar 27 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 12.3.1-3
- Autogenerated by Bloom

* Tue Mar 21 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 12.3.1-2
- Autogenerated by Bloom

