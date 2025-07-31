from setuptools import setup

package_name = 'dynma_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],            # Python 모듈 디렉터리와 일치
    # ---------------------------------------------------------------------
    # 마커(resource/dynma_gui) + package.xml 를 설치해 경고 제거
    # ---------------------------------------------------------------------
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),          # 빈 파일이면 OK
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kjoon',
    maintainer_email='kjoon@example.com',
    description='Mission composer & MURTAP monitor GUI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_gui = dynma_gui.mission_gui:main',  # 모듈.스크립트:함수
        ],
    },
)

