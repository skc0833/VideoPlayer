# VideoPlayer
FFmepg player

## Prerequisites

```
VideoPlayer_ROOT = E:\VideoPlayer
MSYS2_DIR = E:\VideoPlayer\msys64
SDL2_DIR = E:\VideoPlayer\SDL2-devel-2.32.2-VC
FFmpeg_DIR = E:\VideoPlayer\FFmpeg
```

```
// git push 를 위해서는 최초에 SSH-Key 생성해서 등록 필요(Git Bash 쉘에서 실행)
$ ssh-keygen -t ed25519 -C "skc0833@gmail.com" -f ~/.ssh/skc0833_gmail
생성된 C:\Users\skc0833\.ssh\skc0833_gmail.pub 의 내용을 github 우상단(원형 아이콘) Settings / SSH Keys 페이지에서 New SSH key 로 업로드

// 생성된 키를 ssh-agent 에 등록
$ eval "$(ssh-agent -s)"
$ ssh-add ~/.ssh/skc0833_gmail
$ ssh -T git@github.com -v
-->
debug1: Offering public key: /c/Users/skc08/.ssh/id_rsa RSA SHA256:BAxGoIwTAiz8IdkE31RPosGQQgL1AjOOBGL8pw2mDzk
이 아니라 아래와 같이 skc0833@gmail.com 가 출력돼야 함
debug1: Offering public key: skc0833@gmail.com ED25519 SHA256:67budIIhdcNNGnrSFFYn2S6N1XmefKtzIrKPIt1b5Bw agent

// C:\Users\skc0833\.ssh\config 편집
Host github.com-videoplayer
  HostName github.com
  User git
  IdentityFile ~/.ssh/skc0833_gmail
  IdentitiesOnly yes

$ git clone git@github.com:skc0833/VideoPlayer.git
```

### 1. MSYS2

* https://www.msys2.org/ 에서 설치 파일 다운로드 받아 <MSYS2_DIR> 하위에 압축 해제<br/>

* <MSYS2_DIR>\usr\bin\link.exe 파일명을 변경(MSVC 의 link.exe 와 이름 충돌 방지를 위함)<br/>
e.g, link_msys2.exe

* yasm(어셈블리 코드 최적화 및 변환을 지원 도구) 등의 필요 패키지 설치<br/>
```
// <MSYS2_DIR>\msys2.exe 실행 후
$ pacman -S make gcc yasm
-> <MSYS2_DIR>\usr\bin\yasm.exe 에 설치되는 듯함. diffutils 도 설치가 필요할 수 있음

$ pacman -Q gcc yasm diffutils
gcc 13.3.0-1
yasm 1.3.0-3
diffutils 3.10-1

// msys2.exe 터미널 창 종료
```

### 2. SDL2

```
$ cd E:\VideoPlayer
-> 이하 <VideoPlayer_ROOT>

$ cd <VideoPlayer_ROOT>\SDL2-devel-2.32.2-VC
-> 이하 <SDL2_DIR>
```

* SDL2-devel-2.32.2-VC.zip 를 <SDL2_DIR> 에 다운로드
(https://github.com/libsdl-org/SDL/releases)

* e.g, <SDL2_DIR>\SDL2-2.32.2\include\SDL2


### 3. FFmpeg
```
$ cd E:\VideoPlayer\FFmpeg
-> 이하 <FFmpeg_DIR>
$ git clone https://github.com/FFmpeg/FFmpeg.git
$ git checkout release/7.1
-> 현 시점 최신 release branch(527MB), 참고로 기존에는 release/5.1 로 테스트했었음

// git clone -b release/7.1 --single-branch <repo> 도 가능함(460MB)
// 참고로 git clone --depth 1 <repo> 는 다운로드 용량이 많이 줄지만(100MB), branch 이동이 안됨
```


## Build

비주얼스튜디오 개발 프롬프트가 가지고 있는 빌드 환경을 이 MSYS에 전달해주기 위해 아래 명령어를 실행<br/>
(그래야 MSYS에서 gcc 를 실행할때 비주얼스튜디오의 컴파일러와 링커를 찾을 수 있게 됨)

```
1) 윈도우 시작메뉴에서 “x64 Native Tools Command Prompt for VS 2019” 를 실행<br/>
(관리자 모드로 실행이 필요할 수 있음)

2) 다시 아래 명령어로 새로운 콘솔(msys)을 띄운다.<br/>
<MSYS2_DIR>\msys2_shell.cmd -msys -use-full-path<br/>
-> 이렇게 실행된 MSYS 쉘에서 which cl, which link 시 MSVC 경로로 잡혔는지 확인
(e.g, /d/Program Files (x86)/Microsoft Visual Studio/2019/Professional/VC/Tools/MSVC/14.29.30133/bin/HostX64/x64/cl)

3) 빌드
$ cd <FFmpeg_DIR>
$ ./configure --prefix=../install --toolchain=msvc --arch=x86_64 --enable-yasm --disable-x86asm --enable-asm --enable-shared --enable-w32threads --disable-programs --disable-doc --disable-static
-->
화면에 아무것도 표시 안되면서 한참 걸림(5분 이상)
./configure --help 로 전체 옵션 확인 가능함

$ make -j 8     // 실패시 make clean 수행후 재실행
$ make install
```

```
참고로 공식 가이드 내용은(https://trac.ffmpeg.org/wiki/CompilationGuide/MSVC)
(--disable-ffplay 제거, --enable-w32threads 추가가 필요해보임)
./configure --enable-asm --enable-yasm --arch=i386 --disable-ffserver --disable-avdevice --disable-swscale --disable-doc --disable-ffplay --disable-ffprobe --disable-ffmpeg --enable-shared --disable-static --disable-bzlib --disable-libopenjpeg --disable-iconv --disable-zlib --prefix=/c/ffmpeg --toolchain=msvc
```

## MSVC Project 생성

* VS2019 에서 콘솔 프로젝트 생성(빌드시 x64 로 선택)

* 프로젝트 속성 설정
```
// 디버깅 / 환경
PATH=<SDL2_DIR>\SDL2-2.32.2\lib\x64;<FFmpeg_DIR>\libswresample;<FFmpeg_DIR>\libavfilter;<FFmpeg_DIR>\libswscale;<FFmpeg_DIR>\libavformat;<FFmpeg_DIR>\libavutil;<FFmpeg_DIR>\libavcodec;<FFmpeg_DIR>\libavdevice;<FFmpeg_DIR>\libpostproc;%PATH%
-->
PATH=<SDL2_DIR>\SDL2-2.32.2\lib\x64;E:\VideoPlayer\FFmpeg\libswresample;E:\VideoPlayer\FFmpeg\libavfilter;E:\VideoPlayer\FFmpeg\libswscale;E:\VideoPlayer\FFmpeg\libavformat;E:\VideoPlayer\FFmpeg\libavutil;E:\VideoPlayer\FFmpeg\libavcodec;E:\VideoPlayer\FFmpeg\libavdevice;E:\VideoPlayer\FFmpeg\libpostproc;%PATH%

// C/C++ / 일반 / 추가 포함 디렉터리
<FFmpeg_DIR>;<FFmpeg_DIR>\libavdevice;E:\dev\VideoPlayer\SDL2-devel-2.32.2-VC\SDL2-2.32.2\include;$(ProjectDir);%(AdditionalIncludeDirectories)
-->
E:\VideoPlayer\FFmpeg;E:\VideoPlayer\FFmpeg\libavdevice;E:\dev\VideoPlayer\SDL2-devel-2.32.2-VC\SDL2-2.32.2\include;$(ProjectDir);%(AdditionalIncludeDirectories)

// C/C++ / 전처리기
_CRT_SECURE_NO_WARNINGS 추가

// 링커 / 입력 / 추가 종속성
<FFmpeg_DIR>\libavcodec\avcodec.lib;<FFmpeg_DIR>\libavutil\avutil.lib;<FFmpeg_DIR>\libavformat\avformat.lib;<FFmpeg_DIR>\libswscale\swscale.lib;<FFmpeg_DIR>\libavfilter\avfilter.lib;<FFmpeg_DIR>\libswresample\swresample.lib;<FFmpeg_DIR>\libavdevice\avdevice.lib;E:\dev\VideoPlayer\SDL2-devel-2.32.2-VC\SDL2-2.32.2\lib\x64\SDL2.lib;%(AdditionalDependencies)
-->
E:\VideoPlayer\FFmpeg\libavcodec\avcodec.lib;E:\VideoPlayer\FFmpeg\libavutil\avutil.lib;E:\VideoPlayer\FFmpeg\libavformat\avformat.lib;E:\VideoPlayer\FFmpeg\libswscale\swscale.lib;E:\VideoPlayer\FFmpeg\libavfilter\avfilter.lib;E:\VideoPlayer\FFmpeg\libswresample\swresample.lib;E:\VideoPlayer\FFmpeg\libavdevice\avdevice.lib;E:\dev\VideoPlayer\SDL2-devel-2.32.2-VC\SDL2-2.32.2\lib\x64\SDL2.lib;%(AdditionalDependencies)
