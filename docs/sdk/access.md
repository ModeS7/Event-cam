# Getting Metavision SDK access

This page is for the **optional advanced layer** only. The base pipeline in
this repo — driver, renderer, examples, calibration — is built on **OpenEB**
(the open edition of the Metavision SDK) and needs none of what follows. Read
this only if you want to run the closed-source Metavision SDK algorithms
(optical flow, CV, analytics, ...) on top of the event stream, via the
`evk4_sdk_advanced` package.

The full Metavision SDK is **free for Prophesee USB-camera owners** — owning an
EVK4 qualifies you. (Commercial *deployment* needs a separate license; that is
between you and Prophesee and out of scope here.) Access is granted per-account
through Prophesee's private package repository. This page gets you the account
and a token; [install.md](install.md) then installs the SDK.

> **Credentials and the run model:** the token below is a personal credential.
> This project's runtime machines are credential-less, pull-only (see the repo
> README) — so put the token only where you are comfortable holding one (your
> dev / lab machine), and keep it out of the repo (it is `.gitignore`-safe by
> living under `~/.config`, below).

> **The exact UI shifts.** Artifactory's web labels and repo URLs are
> account-specific and change over time; when logged in, JFrog's **Set Me Up**
> panel shows the precise lines for *your* account. Use the steps here to know
> what each is for.

## 1. Request a Prophesee account

Prophesee issues a **Microsoft Entra ID** account that gates the SDK repository.
Fill in the access form — the fields are self-explanatory:

**<https://www.prophesee.ai/resources-access-request/>**

Two fields catch people out:

- **Use an institutional email, not gmail.** The form rejects generic / public
  providers (gmail, qq, etc.) — "No account will be created" for them. Use your
  university or company address.
- **The serial is the `P#####` on the camera** (e.g. `P50000`), printed on the
  housing or PCB. A code starting with `PEK` is the kit code, *not* the serial.
  (It is also a different identifier from the device serial the driver's
  `serial:=` launch argument uses.)

Approval is typically **near-instant** (about a minute). Once approved, log in
to **<https://propheseeai.jfrog.io>** with SAML SSO; your login takes the form
`you@customers.prophesee.ai`.

## 2. Generate a JFrog identity token

The repository authenticates with a token, not your password.

1. On <https://propheseeai.jfrog.io>, click your avatar (top-right) →
   **Edit Profile**.
2. Click **Generate an Identity Token** (give it a description, e.g.
   `metavision-sdk`).
3. **Copy the token immediately** — JFrog shows it **once** and does not save
   it; lose it and you generate a new one. Your `<USER>` is the
   `you@customers.prophesee.ai` login; the token is `<TOKEN>` in the install
   commands.

### Keep the token somewhere safe

Save it before closing the tab — but **never inside a git repository**, where a
stray `git add` would commit it. A password-manager entry is a good home. On the
machine itself, an owner-only file outside any repo works well (and is what
[install.md](install.md) reads from):

```bash
mkdir -p -m 700 ~/.config/prophesee
( umask 077
  read -rsp 'Paste JFrog token, then press Enter: ' TOK; echo
  printf 'PROPHESEE_JFROG_TOKEN=%s\n' "$TOK" > ~/.config/prophesee/jfrog_token )
chmod 600 ~/.config/prophesee/jfrog_token
```

`read -rs` keeps the token out of your shell history; `chmod 600` makes the file
readable only by you. Pull it into a shell when needed:

```bash
source ~/.config/prophesee/jfrog_token   # sets $PROPHESEE_JFROG_TOKEN
```

If the token is ever exposed, revoke it from the same JFrog profile screen and
generate a new one — nothing else has to change.

---

**Next:** [install.md](install.md) — install the SDK (apt on x86, source build
on ARM).
