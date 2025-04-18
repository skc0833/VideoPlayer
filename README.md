# VideoPlayer
FFmepg player

## Prerequisites

```
VideoPlayer_ROOT = d:\VideoPlayer
MSYS2_DIR = d:\msys64
FFmpeg_DIR = d:\VideoPlayer\FFmpeg
```

<details>
<summary>SSH-Key 생성 및 등록</summary>

```
// git push 를 위해서는 최초에 SSH-Key 생성해서 등록 필요(Git Bash 쉘에서 실행)
$ ssh-keygen -t ed25519 -C "skc0833@gmail.com" -f ~/.ssh/skc0833_gmail
생성된 C:\Users\skc0833\.ssh\skc0833_gmail.pub 의 내용을 github 우상단(원형 아이콘) Settings / SSH Keys 페이지에서 New SSH key 로 업로드

// 생성된 키를 ssh-agent 에 등록
$ eval "$(ssh-agent -s)"
$ ssh-add /c/Users/skc0833/.ssh/skc0833_gmail
또는 ssh-add /c/Users/skc08/.ssh/skc0833_gmail
-->
unable to start ssh-agent service, error :1058 에러 발생시,
Win + R → services.msc 입력 후, "OpenSSH Authentication Agent" 를 찾아 우클릭
시작 유형을 "자동", 적용 버튼 클릭 수, 시작 클릭하고 재시도(ssh-agent 가 시작됨)
참고로 ssh-add ~/.ssh/skc0833_gmail 는 못찾고 있다.

$ ssh -T git@github.com -v
--> SSH 연결 상태를 확인
debug1: Offering public key: /c/Users/skc08/.ssh/id_rsa RSA SHA256:BAxGoIwTAiz8IdkE31RPosGQQgL1AjOOBGL8pw2mDzk
id_rsa 이 아니라 아래와 같이 skc0833@gmail.com 가 출력돼야 함
debug1: Offering public key: skc0833@gmail.com ED25519 SHA256:67budIIhdcNNGnrSFFYn2S6N1XmefKtzIrKPIt1b5Bw agent

// C:\Users\skc0833\.ssh\config 편집
Host github.com-videoplayer
  HostName github.com
  User git
  IdentityFile ~/.ssh/skc0833_gmail
  IdentitiesOnly yes

$ git remote set-url origin git@github.com-videoplayer:skc0833/VideoPlayer.git
--> 이걸 안해주면 git@github.com: Permission denied (publickey). 에러
```
</details>

<details>
<summary>Git 설정</summary>

```
// Git 줄바꿈 설정 확인(Windows에서 true or input 으로 설정해야 ^M 표시가 사라짐)
$ git config core.autocrlf
$ git config --global core.autocrlf true
-->
true: 체크아웃 시 CRLF, 커밋 시 LF로 변환
input: 체크아웃 시 CRLF를 쓰지 않고, 커밋 시 LF만 유지함
false(default): 줄바꿈을 건드리지 않음

$ git stash
$ git stash list
$ git stash pop 으로 적용
--> pop: 마지막에 저장된 stash를 현재 작업 디렉토리에 적용 & 해당 stash 항목을 목록에서 제거
git stash apply 는 pop 와 같지만 스태시 목록에서 제거하지 않음(시험삼아 적용해보고 나중에 제거용)
$ git stash pop stash@{0} // 특정 stash 항목을 지정해서 적용

$ git reset HEAD^
--> 가장 최근 커밋을 되돌림(최근 커밋 내용이 unstage 상태로 워킹 디렉토리에 그대로 남음)
$ git reset --hard HEAD^  // 커밋도 삭제하고 변경 내용도 삭제

git push 시에 ERROR: Permission to skc0833/VideoPlayer.git denied to skc0833-cubox-ai. 발생시
위에 생성된 키를 ssh-agent 에 등록, git remote set-url origin ~ 실행 후 재시도
(ssh -T git@github.com -v 로 SSH 연결 상태 확인 필수)
```
</details>

```
$ git clone git@github.com:skc0833/VideoPlayer.git
```

### 1. MSYS2

* https://www.msys2.org/ 에서 설치 파일 다운로드 받아 <MSYS2_DIR> 하위에 압축 해제<br/>
(e.g, msys2-x86_64-20250221.exe 를 다운받아 설치)

* <MSYS2_DIR>\usr\bin\link.exe 파일명을 변경<br/>
(MSVC 의 link.exe 와 이름 충돌 방지를 위해 예를들어 link_msys2.exe 식으로 변경)

* yasm(어셈블리 코드 최적화 및 변환을 지원 도구) 등의 필요 패키지 설치<br/>
```
// <MSYS2_DIR>\msys2.exe 실행 후, 필요 패키지 설치
$ pacman -S make gcc yasm
-> <MSYS2_DIR>\var\lib\pacman\local\make-4.4.1-2 에 설치됨. diffutils 도 설치가 필요할 수 있음

// 설치된 패키지 확인
$ pacman -Q make gcc yasm
make 4.4.1-2
gcc 13.3.0-1
yasm 1.3.0-3

// msys2.exe 터미널 창 종료
```

### 2. SDL2

* SDL 의 releases 페이지에서 SDL2 버전 중, 최신버전을 다운로드<br/>
(https://github.com/libsdl-org/SDL/releases/tag/release-2.32.2 에서 SDL2-devel-2.32.2-VC.zip 다운로드)<br/>
d:\VideoPlayer\SDL2-2.32.2 하위에 include, lib 등이 존재하게 압축해제

* SDL_ttf 도 다운로드(텍스트 자막 파일 출력시 필요)<br/>
https://github.com/libsdl-org/SDL_ttf/releases/tag/release-2.24.0 에서 SDL2_ttf-devel-2.24.0-VC.zip 다운로드

### 3. FFmpeg
```
$ cd <VideoPlayer_ROOT>
$ git clone https://github.com/FFmpeg/FFmpeg.git
$ git checkout release/7.1
-> 현 시점 최신 release branch(527MB), 참고로 기존에는 release/5.1 로 테스트했었음

// git clone -b release/7.1 --single-branch <repo> 도 가능함(460MB)
// 참고로 git clone --depth 1 <repo> 는 다운로드 용량이 많이 줄지만(100MB), branch 이동이 안됨
```


## Build

비주얼스튜디오 개발 프롬프트가 가지고 있는 빌드 환경을 이 MSYS에 전달해주기 위해 아래 명령어를 실행<br/>
(그래야 MSYS에서 gcc 를 실행할때 비주얼스튜디오의 컴파일러와 링커를 찾을 수 있게 됨)

1) 윈도우 시작메뉴에서 “x64 Native Tools Command Prompt for VS 2022” 를 실행<br/>
~~2019 버전에서는 아래 ./configure 시에 Compiler lacks C11 support 에러가 발생하고 있음~~<br/>
2022 버전에서는 ./configure: line 1774: cmp: command not found 에러가 표시되지만 configure는 성공하는 듯함<br/>

2) 다시 아래 명령어로 새로운 콘솔(msys)을 띄운다.<br/>
```
$ <MSYS2_DIR>\msys2_shell.cmd -msys -use-full-path
--> e.g, d:\msys64\msys2_shell.cmd -msys -use-full-path

// 이렇게 실행된 MSYS 쉘에서 which cl, which link 시 MSVC 경로로 잡혔는지 확인
$ which cl
/c/Program Files (x86)/Microsoft Visual Studio/2019/Community/VC/Tools/MSVC/14.25.28610/bin/HostX64/x64/cl
```

3) FFmpeg 빌드
```
$ cd <FFmpeg_DIR>
--> e.g, cd /d/VideoPlayer/FFmpeg/

$ export PKG_CONFIG_PATH=/d/msys64/usr/local/lib/pkgconfig
$ ./configure --prefix=../install --toolchain=msvc --arch=x86_64 --enable-x86asm --enable-asm --enable-shared --enable-w32threads --disable-programs --disable-doc --disable-static --enable-ffmpeg --enable-ffplay --enable-ffprobe

--> 화면에 아무것도 표시 안되면서 한참 걸림(5분 이상)

$ make -j 8     // 실패시 make clean 수행후 재실행
$ make install
--> <VideoPlayer_ROOT>\install\include 폴더에 libpostproc 등이 누락돼 있어 아래 프로젝트 속성에서 "추가 포함 디렉터리" 경로는 <FFmpeg_DIR> 하위로 직접 설정중임
```

<details>
<summary>[참고용] 공식 가이드 내용</summary>

~~공식 가이드 내용은(https://trac.ffmpeg.org/wiki/CompilationGuide/MSVC)
./configure --enable-asm --enable-yasm --arch=i386 --disable-ffserver --disable-avdevice --disable-swscale --disable-doc --disable-ffplay --disable-ffprobe --disable-ffmpeg --enable-shared --disable-static --disable-bzlib --disable-libopenjpeg --disable-iconv --disable-zlib --prefix=/c/ffmpeg --toolchain=msvc
--> 적용시에는 이 명령어에서 --disable-ffplay 제거, --enable-w32threads 추가가 필요해보임!~~
</details>

<details>
<summary>[참고용] FFmepg 명령어</summary>

```
// 구간 잘라내기 명령어
$ ffmpeg.exe -i mov_bbb.mp4 -t 00:00:02 -c copy mov_bbb_2s.mp4
$ ffmpeg.exe -i mov_bbb.mp4 -ss 00:00:00 -t 00:00:02 -c copy mov_bbb_2s.mp4
--> -ss 00:00:00 옵션이 있으면 검은색만 출력되고 있음(TODO: libx264 로 인코딩 필요???)

$ ffmpeg.exe -i mov_bbb.mp4 -ss 00:00:00 -t 00:00:02 -c:v libx264 -c:a aac mov_bbb_2s.mp4
--> Unknown encoder 'libx264' 에러 -> libx264 빌드가 필요하지만 현재 빌드 에러 해결이 안되고 있음
```
</details>

<details>
<summary>[참고용] libx264 빌드(TODO)</summary>

```
// libx264 빌드시 에러(TODO)
$ git clone https://code.videolan.org/videolan/x264.git
$ cd x264
$ ./configure --enable-shared --disable-cli --disable-asm --disable-thread --host=x86_64-w64-mingw32
$ make -j16

// 위에서 FFmpeg configure 시에 아래 에러 발생 중
$ export PKG_CONFIG_PATH=/d/msys64/usr/local/lib/pkgconfig // 소용없는듯
$ export PKG_CONFIG=false // 소용없는듯
$ ./configure ~ --enable-gpl --enable-libx264 --extra-cflags="-I/d/msys64/usr/local/include" --extra-ldflags="-L/d/msys64/usr/local/lib" --pkgconfigdir=/d/msys64/usr/local/lib/pkgconfig
 옵션 추가시
--> ERROR: x264 not found using pkg-config -> TODO: 해결책 못 찾음
```
</details>


## MSVC Project 생성

* <VideoPlayer_ROOT>\VideoPlayer\VideoPlayer.sln 를 오픈하면 빌드, 실행(F5)이 성공해야 함<br/>
--> e.g, D:\VideoPlayer\VideoPlayer\VideoPlayer.sln

* cmd 터미널에서 실행할 경우
```
1) PATH 환경변수를 설정하거나
--> set PATH=D:\VideoPlayer\SDL2-2.32.2\lib\x64;D:\VideoPlayer\install\bin;%PATH%
2) 또는 필요한 dll 파일들을 실행파일 위치에 복사한 후, 실행
--> D:\VideoPlayer\SDL2-2.32.2\lib\x64\SDL2.dll, D:\VideoPlayer\install\bin\*.dll 을 실행파일 위치로 복사

> cd D:\VideoPlayer\VideoPlayer\x64\Debug
> VideoPlayer.exe D:\VideoPlayer\VideoPlayer\data\mov_bbb.mp4
```

* 이하는 MSVC Project 생성 및 설정 내용임

* VS2022 에서 C++ 콘솔앱 프로젝트 생성(빌드시 x64 로 선택)<br/>
프로젝트 이름: VideoPlayer, 위치: <VideoPlayer_ROOT>\VideoPlayer (e.g, d:\VideoPlayer\VideoPlayer)<br/>
솔루션 및 프로젝트를 같은 디렉토리에 배치

* VS2022 에서 빌드시 error MSB8020: v143 에러가 발생하면, 프로젝트 속성의 플랫폼 도구 집합을 Visual Studio 2019 (v142) 로 설정

* 프로젝트 속성 설정
```
// 디버깅 / 환경
PATH=$(ProjectDir)\..\SDL2-2.32.2\lib\x64;$(ProjectDir)\..\install\bin;%PATH%

// C/C++ / 일반 / 추가 포함 디렉터리
$(ProjectDir)\..\FFmpeg;$(ProjectDir)\..\FFmpeg\fftools;$(ProjectDir)\..\SDL2-2.32.2\include;$(ProjectDir);%(AdditionalIncludeDirectories)

// C/C++ / 전처리기
_CRT_SECURE_NO_WARNINGS 추가

// 링커 / 입력 / 추가 종속성
$(ProjectDir)\..\install\bin\avcodec.lib;$(ProjectDir)\..\install\bin\avutil.lib;$(ProjectDir)\..\install\bin\avformat.lib;$(ProjectDir)\..\install\bin\swscale.lib;$(ProjectDir)\..\install\bin\avfilter.lib;$(ProjectDir)\..\install\bin\swresample.lib;$(ProjectDir)\..\install\bin\avdevice.lib;$(ProjectDir)\..\SDL2-2.32.2\lib\x64\SDL2.lib;%(AdditionalDependencies)
