REMOTE_ADDR="ubuntu-virt"
REMOTE_USER="derek"
DESTINATION="/home/${REMOTE_USER}/"

REPO_ROOT=$(git rev-parse --show-toplevel)
REPO_NAME=${REPO_ROOT##*/}

rsync --recursive --delete --archive --compress ${REPO_ROOT} ${REMOTE_USER}@${REMOTE_ADDR}:${DESTINATION}
