mise: STM32 기반 미세먼지 및 온습도 측정 시스템
이 프로젝트는 STM32 마이크로컨트롤러(Nucleo-F401RE 보드)를 활용하여 미세먼지(PM)와 온습도 데이터를 측정하고, 이를 UART를 통해 Python 스크립트로 전송한 후 Twilio API를 사용하여 SMS로 알림을 보내는 임베디드 시스템입니다.

주요 기능
센서 데이터 수집: 미세먼지 및 온습도 센서를 통해 환경 데이터를 측정합니다.

데이터 통신: I2C 및 UART 프로토콜을 사용하여 센서와 마이크로컨트롤러 간의 통신을 구현합니다.

Python 연동: UART를 통해 수신된 데이터를 Python 스크립트로 처리합니다.

SMS 알림: Twilio API를 활용하여 특정 조건에 따라 SMS 알림을 전송합니다.

사용된 하드웨어 및 소프트웨어
마이크로컨트롤러: Nucleo-F401RE (STM32F401RE)

개발 환경: STM32CubeIDE (초기에는 STM32CubeMX 사용)

센서: 미세먼지 센서(단일 핀 측정 방식), 온습도 센서

통신 방식: I2C, UART

Python 스크립트: 데이터 수신 및 Twilio API 연동

프로젝트 구조
plaintext
복사
편집
mise/
├── Core/                # 메인 펌웨어 소스 코드
├── Drivers/             # STM32 드라이버 및 HAL 라이브러리
├── Debug/               # 디버그 및 빌드 아티팩트
├── FFT_data/            # FFT 관련 데이터 (필요시)
├── .settings/           # IDE 설정 파일
├── .cproject            # Eclipse 프로젝트 설정
├── .mxproject           # STM32CubeMX 프로젝트 파일
├── .project             # Eclipse 프로젝트 메타데이터
├── STM32F401RETX_FLASH.ld  # 플래시 메모리 링커 스크립트
├── STM32F401RETX_RAM.ld    # RAM 링커 스크립트
├── f401re_timer1ms_uart_dht11_dust_1.ioc   # CubeMX 설정 파일
├── f401re_timer1ms_uart_dht11_dust_1.launch # IDE 실행 설정
└── README.md            # 프로젝트 설명서
주의사항
센서 연결: 센서와 보드 간의 연결 시 핀 번호 및 전압 레벨을 정확히 확인하시기 바랍니다.

미세먼지 센서: 현재 사용된 미세먼지 센서는 단일 핀으로 측정하는 방식이며, 향후 정확도를 높이기 위해 더 정밀한 센서로 교체를 고려하고 있습니다.

Twilio 연동: Twilio API를 사용하기 위해서는 별도의 계정 및 인증 정보가 필요합니다.

데모 영상
프로젝트의 최종 발표 및 데모 영상은 아래 링크에서 확인하실 수 있습니다:
https://youtu.be/oGgFmyQPvDk?si=ujOeshRdok8dzbhv


