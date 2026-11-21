# service/static: what the deployed twin and Lab console consist of

The twin and the Lab console are static files on S3 behind CloudFront. This
directory is the manifest of those files and the script that publishes them.

| File | Purpose |
|---|---|
| `assets.json` | Every file served, as `{src, key, type, cache}` rows: the repo path, the S3 key it lands on, its content type, and a cache policy (`none` = `no-cache, must-revalidate`, `day` = `public, max-age=86400`). **Order matters**: the `web-twin/js/` modules upload before `app.js`, and `index.html` after the modules, so a phone never gets a page that imports a key not yet in the bucket. `control/lab.html` appears three times, at the extensionless keys `admin`, `metrics` and `lab` |
| `sync.sh` | Puts each object with `aws s3api put-object` (not `s3 sync`, which guesses content types and would make the browser download the extensionless Lab pages), then invalidates `/*` and **waits for `Completed`** |

## Run

```bash
bash service/static/sync.sh <static-bucket> <distribution-id> [region]   # region defaults to us-east-2
```

Both values are the serverless stack's outputs (`aws cloudformation
describe-stacks --stack-name vision-picar-serverless --query
'Stacks[0].Outputs'`). The script refuses to sync an empty manifest, stops
on a listed file that is missing, and aborts if it uploaded fewer objects
than the manifest lists.

A `web-twin/` change is not shipped until the invalidation reports
`Completed` (`CLAUDE.md` section 6). **Each deploy needs the user's go**: a
cloud session holds an IAM user that can put objects, invalidate and
describe stacks, and nothing more.

A new page asset goes in `assets.json`; `tests/test_static_assets.py`
checks that every file `web-twin/index.html` and `control/lab.html`
reference is listed. `sync.sh` uploads only the paths in the manifest, so
a README in this directory is not published.

## Where the rest is written

`docs/engineering/operations/ENGINEERING.md` (the static site, the deploy
procedure and parity check); `docs/guides/FEATURES.md` for the twin.
