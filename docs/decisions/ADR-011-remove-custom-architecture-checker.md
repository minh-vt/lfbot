# ADR-011: Bo custom architecture checker

**Status:** Accepted; thay phan custom architecture gate cua [ADR-009](ADR-009-scoped-robot-native-runtime.md) va [ADR-010](ADR-010-preserve-ui-and-name-sensor-contracts.md).

## Context

Checker AST/path/build membership va fixtures lam tang chi phi bao tri va rang buoc layout. Theo yeu cau chu du an, bo checker thay vi tiep tuc mo rong no.

## Decision

- Xoa custom architecture checker, fixtures va recipe; khong tao tool thay the.
- Giu compiler/CMake, clang-tidy, sanitizer va tests hanh vi qua `just`.
- Test entrypoints nam truc tiep trong `firmware/test/`; native CMake frontend nam o root repo.

## Consequences

Ranh gioi ownership/core/platform van la yeu cau, duoc giu bang interface C++, build dependencies va review; khong con AST/path gate tu dong cam moi vi pham. Build xanh khong chung minh robot hoan thanh track, timing hay an toan phan cung.
