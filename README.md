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
// git push 를 위해서는 최초에 SSH-Key 생성해서 등록 필요
> ssh-keygen -t ed25519 -C "skc0833@gmail.com"
생성된 C:\Users\skc0833/.ssh/id_ed25519.pub 의 내용을 github 우상단(원형 아이콘) Settings / SSH Keys 페이지에서 New SSH key 로 업로드

// SSH 방식으로 remote URL 변경
> git remote -v
origin  https://github.com/skc0833/VideoPlayer.git (fetch)
<-- 이렇게 표시되면 HTTPS 방식으로 연결된 상태이며, SSH 방식으로 변경이 필요함
(아니면 로그인창이 뜨고, 로그인해도 계속 실패함. HTTPS에선 SSH 키가 아니라 토큰을 요구함)

> git remote set-url origin git@github.com:skc0833/VideoPlayer.git
> git remote -v
> origin  git@github.com:skc0833/VideoPlayer.git (fetch)
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
-> 현 시점 최신 release branch(527MB)

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
TODO: 이하 경로들 VideoPlayer_ROOT 하위로 수정 필요!!!

// 디버깅 / 환경
PATH=E:\dev\VideoPlayer\SDL2-devel-2.32.2-VC\SDL2-2.32.2\lib\x64;E:\skc_work\VideoEditor\FFmpeg\libswresample;E:\skc_work\VideoEditor\FFmpeg\libavfilter;E:\skc_work\VideoEditor\FFmpeg\libswscale;E:\skc_work\VideoEditor\FFmpeg\libavformat;E:\skc_work\VideoEditor\FFmpeg\libavutil;E:\skc_work\VideoEditor\FFmpeg\libavcodec;E:\skc_work\VideoEditor\FFmpeg\libavdevice;E:\skc_work\VideoEditor\FFmpeg\libpostproc;%PATH%

// C/C++ / 일반 / 추가 포함 디렉터리
E:\skc_work\VideoEditor\FFmpeg;E:\skc_work\VideoEditor\FFmpeg\libavdevice;E:\dev\VideoPlayer\SDL2-devel-2.32.2-VC\SDL2-2.32.2\include;$(ProjectDir);%(AdditionalIncludeDirectories)

// C/C++ / 전처리기
_CRT_SECURE_NO_WARNINGS 추가

// 링커 / 입력 / 추가 종속성
E:\skc_work\VideoEditor\FFmpeg\libavcodec\avcodec.lib;E:\skc_work\VideoEditor\FFmpeg\libavutil\avutil.lib;E:\skc_work\VideoEditor\FFmpeg\libavformat\avformat.lib;E:\skc_work\VideoEditor\FFmpeg\libswscale\swscale.lib;E:\skc_work\VideoEditor\FFmpeg\libavfilter\avfilter.lib;E:\skc_work\VideoEditor\FFmpeg\libswresample\swresample.lib;E:\skc_work\VideoEditor\FFmpeg\libavdevice\avdevice.lib;E:\dev\VideoPlayer\SDL2-devel-2.32.2-VC\SDL2-2.32.2\lib\x64\SDL2.lib;%(AdditionalDependencies)

