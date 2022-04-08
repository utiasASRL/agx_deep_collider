from setuptools import setup


setup(
    name='crowdnav',
    version='0.1.1',
    packages=[
        'simple_nav',
        'simple_nav.policy',
        'simple_nav.utils',
        # 'simpe_nav.configs',
        'crowd_nav',
        'crowd_nav.configs',
        'crowd_nav.policy',
        'crowd_nav.utils',
        'crowd_sim',
        'crowd_sim.envs',
        'crowd_sim.envs.policy',
        'crowd_sim.envs.utils',
    ],
    install_requires=[
        'gitpython',
        'gym==0.10.0',
        'matplotlib',
        'numpy',
        'scipy',
        'torch',
        'torchvision',
    ],
    extras_require={
        'test': [
            'pylint',
            'pytest',
        ],
    },
)
