from setuptools import setup

setup(name='uav_mapping', version='0.1.0', packages=['uav_mapping'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/uav_mapping']),
                  ('share/uav_mapping', ['package.xml']),
                  ('share/uav_mapping/config', ['config/rmuc_2025_prior.pgm', 'config/rmuc_2025_prior.yaml',
                                                'config/rmuc_2025_prior.metadata.json', 'config/params.yaml'])],
      install_requires=['setuptools', 'numpy'], zip_safe=True,
      maintainer='rm27-uav', maintainer_email='team@example.com', license='MIT',
      entry_points={'console_scripts': ['rolling_mapper = uav_mapping.rolling_mapper:main',
                                        'prior_mapper = uav_mapping.prior_mapper:main']})
