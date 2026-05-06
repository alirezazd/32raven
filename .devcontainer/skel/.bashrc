# 32Raven devcontainer .bashrc
# Deployed to /home/builder/ by .devcontainer postCreateCommand if missing.
# Personal tweaks: edit ~/.bashrc inside the container (it lives on the host
# at .docker/home/.bashrc and is gitignored).

# If not running interactively, do nothing.
case $- in
  *i*) ;;
  *) return ;;
esac

# Pull in system-wide defaults (shopt checkwinsize, histappend hint, etc.)
[ -r /etc/bash.bashrc ] && . /etc/bash.bashrc

# History
HISTSIZE=10000
HISTFILESIZE=20000
HISTCONTROL=ignoreboth:erasedups
shopt -s histappend
shopt -s checkwinsize
shopt -s autocd 2>/dev/null
shopt -s cdspell 2>/dev/null
shopt -s dirspell 2>/dev/null

# Colored prompt: user@host:cwd, branch if in a git repo, $ on a new line.
__prompt_git_branch() {
  local b
  b=$(git symbolic-ref --quiet --short HEAD 2>/dev/null) || \
    b=$(git rev-parse --short HEAD 2>/dev/null) || return
  printf ' (%s)' "$b"
}
PS1='\[\e[1;32m\]\u@\h\[\e[0m\]:\[\e[1;34m\]\w\[\e[33m\]$(__prompt_git_branch)\[\e[0m\]\n\$ '

# Color aliases.
if command -v dircolors >/dev/null 2>&1; then
  eval "$(dircolors -b)"
fi
alias ls='ls --color=auto'
alias ll='ls -lh --color=auto'
alias la='ls -lAh --color=auto'
alias l='ls -CF --color=auto'
alias grep='grep --color=auto'
alias egrep='egrep --color=auto'
alias fgrep='fgrep --color=auto'
alias diff='diff --color=auto'
alias ip='ip --color=auto'

# bash-completion (installed via Dockerfile).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  fi
fi

# Make less behave for non-text input.
export LESS='-R --mouse'
export PAGER=less

# 32Raven helpers.
alias mk='make'
alias build-stm32='make stm32'
alias build-esp32='make esp32'
