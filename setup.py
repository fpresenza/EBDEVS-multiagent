# -*- coding: utf-8 -*-
import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="multirobot_ebdevs",
    version="0.0.1",
    license='GPLv3',
    author="J. Francisco Presenza and Ezequiel Pecker-Marcosig",
    author_email='jpresenza@fi.uba.ar, epecker@fi.uba.ar',
    description=u'\
        Emergent Behavior DEVS simulator for multirobot systems.',
    long_description=long_description,
    long_description_content_type="text/rst",
    packages=setuptools.find_packages(),
    package_data={'multirobot_ebdevs': ['config/*.yaml']},
    classifiers=[
        "Programming Language :: Python :: 3.6",
        "License :: OSI Approved :: GNU General Public License v3  \
        or later (GPLv3+)",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
