project('A Star', 'cpp', version : '1.0.0', default_options: ['cpp_std=c++2a'])

inc = include_directories('include')

subdir('include')
subdir('src')

pkg_mod = import('pkgconfig')
pkg_mod.generate(libraries : rttstarlib,
                 version : '1.0',
                 name : 'librttstar',
                 filebase : 'rttstar',
                 description : 'A Library for RTT Star.')


exe = executable('example', 'main.cpp',
                    include_directories : inc,
                    link_with : rttstarlib)