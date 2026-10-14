# 공유·기여 안내

저장소: `lee-robot-lab/rover-mapping`, private. 초대받은 GitHub 계정으로 접근한다. 사용 설명서는 [user-guide.md](user-guide.md), 설치는 [installation.md](installation.md)다.

## 소유자의 팀원 초대

초대 대상 GitHub 아이디를 확인한 뒤 GitHub 저장소의 **Settings → Collaborators → Add people**에서 초대한다. 읽기만 필요한 인수인계 대상과 코드 수정 담당자를 구분해 권한을 정한다. 이번 초기 준비에서 임의 팀원을 초대하지 않았다.

대상이 정해지지 않아 초대는 남아 있다. private URL만 전달해도 초대받지 않은 계정은 내용을 볼 수 없다.

## 팀원이 처음 할 일

```bash
git clone git@github.com:lee-robot-lab/rover-mapping.git
cd rover-mapping
```

1. 설치 설명서로 Python 환경을 준비한다.
2. 합성 GUI와 terrain demo를 실행한다.
3. README의 fake SPI/매핑 회귀를 실행한다.
4. 실제 작업을 할 사람은 native/Pi runtime·필요 RUN을 별도로 인계받는다.
5. 인수인계서의 남은 과제를 확인하고 실기 검증과 합성/재생 결과를 구분한다.

## 변경 작업

```bash
git switch -c feature/작업명
# 관련 코드/문서를 수정하고 테스트
git add 명시한_파일
git commit -m "변경 내용"
git push -u origin HEAD
```

기본 main을 임의 force-push하지 않는다. 촬영 원본·비밀정보·venv·build binary는 Git에 넣지 않는다. 실험은 원본을 보존한 별도 RUN에서 하고 재현 조건·설정·검증 범위를 기록한다. motor/통신 코드를 바꾸면 fake/시뮬레이션부터 확인한다.

## 현재 작업 폴더 관계

- 원본 연구/실기 workspace: `/home/su/rover`.
- 이번 공유 체크아웃: `/home/su/rover-mapping`.
- 원본은 빈 `.git` 디렉터리만 있어 Git 저장소가 아니었으며, 원본 전체를 새 레포로 초기화하지 않았다.
- 공유본 source hash는 [source-snapshot.json](source-snapshot.json)에 있다. 운영 소스는 그대로이며 테스트2파일은 native 미설치 조건에 맞게 fixture를 정리했다. 원본/shared hash를 함께 보존했다. 원본 수정은 공유본에 자동 동기화되지 않으므로 필요한 파일 변경을 확인해 옮기고 같은 회귀를 실행한다.

사용자에게 초대 대상 아이디를 받으면 해당 대상의 접근 권한을 연결할 수 있다. 인증 토큰/SSH private key를 저장소 파일로 공유하지 않는다.
