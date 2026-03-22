# MC68020 에뮬레이터 개선 계획

## 현 상태 요약

- 코드 규모: 8,517 LOC / 33 파일 / 95개 명령어 핸들러
- 테스트: 108 assertions / 43 테스트 함수, 전부 통과
- 파이프라인: B/C/E 3단 오버랩 모델 구현 완료
- 명령어 캐시: 64엔트리 직접매핑 구현 완료
- **평가: 교육용 시뮬레이터 수준. 실제 소프트웨어 실행 불가.**

## 핵심 문제

실제 68020 바이너리(AmigaOS, Atari ST 커널 등) 실행 시 즉시 실패하는 원인:

1. 버스 에러 SSW에 FC/크기/RMW 비트 누락 → OS 메모리 관리자 동작 불가
2. 디스패치 테이블 충돌 가능성 미검증 (65536 슬롯)
3. EA 해석기 엣지케이스 미처리 (An 바이트 쓰기 등)
4. 테스트가 합성 코드만 검증, 실제 바이너리 미검증
5. BKPT 등 누락 명령어

---

## Phase 8: 버스 에러 SSW 완성

**목표**: 실제 예외 핸들러가 SSW를 읽어 오류 원인을 파악할 수 있도록 한다.

### 8A. SSW 비트 필드 완성
**파일**: `src/bus/m68020_bus.c`, `src/bus/m68020_bus_error.c`

현재 SSW에 RW 비트(bit 8)만 설정됨. 나머지 필수 비트:

| 비트 | 이름 | 현재 | 수정 |
|------|------|------|------|
| 12-10 | FC2-FC0 | 항상 0 | 오류 발생 시의 Function Code 기록 |
| 8 | RW | 구현됨 | 유지 |
| 6 | RM | 항상 0 | read-modify-write 사이클 표시 (CAS, TAS 등) |
| 5 | DF | 항상 0 | 데이터 페치 vs 명령어 페치 구분 |
| 3-0 | SIZE | 항상 0 | 접근 크기 (00=byte, 01=word, 10=long) |

**수정 방법**:
- `exception_bus_error()` 호출부 27곳에서 FC, size 정보를 함께 전달
- `cpu_read_*`/`cpu_write_*`에서 SSW를 구성하여 전달
- `cpu_fetch_word`에서는 DF=0 (명령어 페치), 데이터 접근에서는 DF=1

### 8B. exception_bus_error 시그니처 확장
**파일**: `include/m68020_internal.h`, `src/bus/m68020_bus_error.c`

```c
// 현재
void exception_bus_error(M68020State *cpu, u32 fault_addr, bool is_write, u16 ssw);

// 변경
void exception_bus_error(M68020State *cpu, u32 fault_addr,
                         FunctionCode fc, BusSize size,
                         bool is_write, bool is_rmw);
```

SSW를 함수 내부에서 조립:
```c
u16 ssw = ((u16)fc << 10)
        | (is_write ? 0 : 0x0100)
        | (is_rmw ? 0x0040 : 0)
        | (is_ifetch ? 0 : 0x0020)  // DF bit
        | (size_bits);
```

### 8C. 테스트
- `test_bus_error_ssw_fc`: 데이터 접근 버스 에러 시 SSW의 FC 비트 확인
- `test_bus_error_ssw_size`: byte/word/long 접근별 SSW SIZE 비트 확인
- `test_bus_error_ssw_rw`: 읽기/쓰기 구분 확인

---

## Phase 9: 디스패치 테이블 무결성

**목표**: 65536개 opcode 슬롯에 올바른 핸들러가 매핑되어 있는지 검증한다.

### 9A. 디스패치 테이블 검증 도구
**파일**: `tests/integration/test_dispatch.c` (신규)

65536개 슬롯을 순회하며:
1. 모든 슬롯이 NULL이 아닌지 확인 (handler_illegal 포함)
2. 알려진 유효 opcode가 올바른 핸들러를 가리키는지 확인
3. 알려진 무효 opcode가 handler_illegal을 가리키는지 확인

### 9B. Shift/Bitfield 오버랩 정리
**파일**: `src/instructions/m68020_shift.c`

현재 `0xE000-0xEFFF` 전체에 `do_shift`를 설치한 후 bitfield가 덮어씀.
→ shift 설치 범위를 bitfield 영역(`0xE8C0-0xEFFF` 중 해당 패턴)에서 제외.

### 9C. 충돌 감지 테스트
- `test_dispatch_no_unintended_overlap`: 선택된 100개 opcode에 대해 올바른 핸들러 확인
- `test_known_illegal_opcodes`: 알려진 무효 opcode가 illegal 핸들러를 가리키는지 확인

---

## Phase 10: EA 해석기 엣지케이스

**목표**: MC68020 매뉴얼에 명시된 모든 EA 규칙을 준수한다.

### 10A. An 바이트 쓰기 거부
**파일**: `src/cpu/m68020_ea.c`

68020 사양: 주소 레지스터에 대한 바이트 연산은 불법.
현재 `ea_write(EAK_An, SIZE_BYTE, ...)` 가 예외 없이 실행됨.
→ SIZE_BYTE + EAK_An 조합에서 `VEC_ILLEGAL_INSN` 발생하도록 수정.

### 10B. ea_resolve 유효 모드 검증 강화
- 각 명령어 그룹별 유효 EA 모드 테이블 작성
- 무효 EA 모드에 대해 illegal instruction 발생 확인

### 10C. Post-increment/Pre-decrement 정확성
- `MOVE.W (A0)+,(A1)+` 같은 이중 post-increment 검증
- MOVEM의 predecrement 순서 재검증 (Phase 5에서 수정했지만 추가 엣지케이스)

### 10D. 테스트
- `test_ea_byte_write_an_illegal`: MOVE.B src,An → illegal exception
- `test_ea_double_postinc`: (A0)+ 소스와 (A1)+ 목적지 동시 사용
- `test_ea_memory_indirect_full`: 풀 포맷 EA (base disp + outer disp + 인덱스)

---

## Phase 11: 누락 명령어 구현

### 11A. BKPT (브레이크포인트)
**파일**: `src/instructions/m68020_misc.c`

- 인코딩: `0100 1000 0100 1 vvv` (0x4848 | vector)
- 동작: CPU가 BKPT 사이클을 외부에 알리고, 외부 하드웨어가 응답
- 에뮬레이터에서는: `M68020BusInterface`에 `bkpt` 콜백 추가 또는 `VEC_ILLEGAL_INSN` 발생

### 11B. 기타 검증
- PACK/UNPK: 이미 구현됨, 32비트 입력 엣지케이스 테스트 추가
- ABCD/SBCD: 이미 구현됨, 캐리 전파 엣지케이스 테스트 추가
- CALLM/RTM: 스텁 상태, 실제 사용 빈도가 매우 낮으므로 우선순위 낮음

---

## Phase 12: 실제 바이너리 검증 인프라

**목표**: 합성 테스트를 넘어 실제 68020 코드로 검증한다.

### 12A. 기존 테스트 스위트 연동
- **Musashi 테스트 벡터**: 오픈소스 68020 에뮬레이터의 테스트 데이터 활용
- **68000-testgen**: 자동 생성된 명령어별 테스트 케이스
- 각 테스트: 초기 레지스터 상태 → 명령어 실행 → 최종 상태 비교

### 12B. 비교 테스트 프레임워크
**파일**: `tests/comparison/test_compare.c` (신규)

```c
typedef struct {
    u32 d[8], a[8], pc, sr;  // 초기 상태
    u8  code[32];             // 실행할 명령어
    u32 d_exp[8], a_exp[8];   // 기대 결과
    u16 sr_exp;
} TestVector;
```

외부 파일에서 테스트 벡터를 로드하여 대량 실행.

### 12C. 사이클 비교
- MAME 68020 코어의 명령어별 사이클 카운트와 비교
- 허용 오차 범위 설정 (파이프라인 모델이 근사치이므로)

---

## Phase 13: 디버깅 도구

### 13A. 디스어셈블러
**파일**: `src/debug/m68020_disasm.c` (현재 스텁)

- 16비트 opword → 니모닉 문자열 변환
- EA 모드별 피연산자 포맷팅
- 확장 워드 디코딩 (즉치값, 변위 등)
- 예상 규모: ~600 LOC

### 13B. 실행 트레이스
**파일**: `src/debug/m68020_trace.c` (현재 스텁)

- 명령어 실행 로그: PC, opword, 니모닉, 레지스터 변경, 사이클
- 링 버퍼 방식으로 최근 N개 명령어 기록
- 버스 에러 / 예외 발생 시 자동 덤프

---

## Phase 14: 시스템 통합 준비

### 14A. 메모리 맵 지원
- 현재: 단일 flat_read/flat_write 콜백
- 개선: 주소 범위별 핸들러 등록 (ROM, RAM, MMIO 영역 분리)
- `M68020BusInterface`에 메모리 맵 레이어 추가 (선택적)

### 14B. DMA / 버스 중재
- 외부 DMA 요청 시 CPU 버스 양보 모델링
- `m68020_bus_request()` / `m68020_bus_grant()` API

### 14C. 실시간 성능 최적화
- 현재: 디스패치 테이블 룩업 (빠름)
- 개선 가능: JIT 힌트, 핫 루프 감지, 블록 단위 실행

---

## 우선순위 및 의존성

```
Phase 8 (SSW) ────────────────┐
Phase 9 (디스패치 검증) ──────┤
Phase 10 (EA 엣지케이스) ─────┼──→ Phase 12 (바이너리 검증)
Phase 11 (누락 명령어) ───────┘         ↓
                                  Phase 14 (시스템 통합)
Phase 13 (디버깅 도구) ──────────────→ (독립, 언제든 병행 가능)
```

| Phase | 우선순위 | 예상 규모 | 사전 조건 |
|-------|----------|-----------|-----------|
| 8 (SSW) | **최고** | 중 (~200 LOC) | 없음 |
| 9 (디스패치) | **높음** | 소 (~150 LOC) | 없음 |
| 10 (EA) | **높음** | 소 (~100 LOC) | 없음 |
| 11 (누락 명령어) | 중 | 소 (~50 LOC) | 없음 |
| 12 (바이너리 검증) | **최고** | 대 (~500 LOC + 외부 데이터) | 8, 9, 10 |
| 13 (디버깅 도구) | 중 | 대 (~700 LOC) | 없음 |
| 14 (시스템 통합) | 낮음 | 대 | 12 |

## 성공 기준

- Phase 8-11 완료 후: 알려진 구조적 결함 없음
- Phase 12 완료 후: Musashi 테스트 벡터 95% 이상 통과
- Phase 13 완료 후: 실행 추적으로 실패 원인 즉시 진단 가능
- Phase 14 완료 후: Amiga Kickstart ROM 초기 부팅 시퀀스 실행 가능
