from setuptools import setup

with open('pypi_desc.md') as f:
    long_description = f.read()


setup(
    name='pymata-cpx',
    version='1.0',
    packages=['pymata_cpx'],
    install_requires=['pyserial'],
    url='https://mryslab.github.io/pymata-cpx/',
    download_url='https://github.com/MrYsLab/pymata-cpx',
    license='GNU Affero General Public License v3 or later (AGPLv3+)',
    author='Alan Yorinks',
    author_email='MisterYsLab@gmail.com',
    description='A Python API For The Circuit Playground Express',
    long_description=long_description,
    long_description_content_type='text/markdown',
    keywords=['Circuit Playground Express', 'Python',],
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Environment :: Other Environment',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'License :: OSI Approved :: GNU Affero General Public License v3 or later (AGPLv3+)',
        'Operating System :: OS Independent',
        'Programming Language :: Python :: 3 :: Only',
        'Topic :: Utilities',
        'Topic :: Education',
    ],
)

