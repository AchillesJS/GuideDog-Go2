# Team and collaboration

GuideDog-Go2 is a **two-developer project**. The team intends a balanced 50/50 workload and shared technical understanding. The following ownership split is a **proposal**, not yet a binding assignment:

| Area | Initial lead proposal | Joint responsibilities |
| --- | --- | --- |
| Visual perception, detection/tracking, local navigation | Developer B | Interfaces, tests, reviews |
| LiDAR geometry, world model, sensor fusion, safety | Developer A | Interfaces, tests, reviews |

Both developers participate in architecture, integration, documentation, demos, and acceptance criteria. Leads do not imply exclusive access to a component.

## Suggested workflow

1. Discuss interfaces and acceptance criteria before building dependent components.
2. Create a GitHub Issue for each concrete task.
3. Develop on `feature/<issue>-short-name`, not directly on `main`.
4. Open pull requests with reproduction steps and observable results.
5. Request review from the other developer before merge.
6. Update docs whenever assumptions, topics, or dependencies change.

## Boundaries

- No proprietary material, code, models, datasets, or confidential information from employers, customers, or other projects.
- Do not vendor the third-party Go2 simulator without checking its license and a specific reason.
- Do not claim physical guide-dog functionality based on simulation tests.
- Original GuideDog-Go2 code uses Apache License 2.0. Respect third-party licenses and maintain appropriate attribution.
