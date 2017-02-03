Name:    gst-plugins-scaler
Version: 1.0.3
Release: 1
License: LGPLv2+
Summary: nexell scaler gstreamer plugin
Group: Development/Libraries
Source:  %{name}-%{version}.tar.gz

BuildRequires:	pkgconfig automake autoconf libtool
BuildRequires:	pkgconfig(glib-2.0)
BuildRequires:	gstreamer1-devel
BuildRequires:	glibc-devel
BuildRequires:	gstreamer1-plugins-base-devel
BuildRequires:	nx-scaler-devel
BuildRequires:	nx-drm-allocator-devel
BuildRequires:	nx-gst-meta-devel

Requires(post): /sbin/ldconfig
Requires(postun): /sbin/ldconfig

%description
Nexell scaler gstreamer plugin

%prep
%setup -q

%build
autoreconf -v --install || exit 1
%configure --with-extrapath=%{_prefix} --with-extrapath_lib=%{_libdir} \
	--with-extrapath_include=%{_includedir}
make %{?_smp_mflags}

%postun -p /sbin/ldconfig

%install
rm -rf %{buildroot}
make install DESTDIR=%{buildroot}

find %{buildroot} -type f -name "*.la" -delete

%files
%{_libdir}/gstreamer-1.0/libgstnxscaler.so
%{_libdir}/gstreamer-1.0/libgstnxscaler.so.*
%license LICENSE.LGPLv2+
