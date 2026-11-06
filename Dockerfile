FROM nginxinc/nginx-unprivileged:1.28.2-alpine@sha256:7377697a821c131a924a7105fafbe7414db4e9fcc77a6f08f776f33f141ec3f8

ARG SOURCE_REVISION=unknown
LABEL org.opencontainers.image.title="Flock nature showcase" \
      org.opencontainers.image.source="https://github.com/martines3000/autonomous-characters-3d" \
      org.opencontainers.image.revision="${SOURCE_REVISION}"

COPY --chown=101:101 web/nginx.conf /etc/nginx/nginx.conf
COPY --chown=101:101 dist/ /srv/flock/

USER 101:101
EXPOSE 8080
STOPSIGNAL SIGQUIT
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
  CMD wget -q -O /dev/null http://127.0.0.1:8080/healthz || exit 1
ENTRYPOINT ["nginx"]
CMD ["-g", "daemon off;"]
