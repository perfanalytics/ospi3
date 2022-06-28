#!/usr/bin/env python2

from distutils.core import setup

setup(
    name='ospi',
    description='This library contains scripts for working with OpenSim files and pinocchio software.',
    version='1.0.3',
    packages=['ospi'],
    keywords='biomechanics OpenSim pinocchio',
    url='https://github.com/gepetto/ospi/',
    author='Galo MALDONADO',
    author_email='galo.maldonado@laas.fr',
    license='LGPL',
    install_requires=['numpy','pandas','scipy'],
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Science/Research',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3.9',
    ],
)
