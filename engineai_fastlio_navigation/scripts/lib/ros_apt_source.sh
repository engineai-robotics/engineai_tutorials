#!/usr/bin/env bash
# Reuse existing ROS 2 repositories; never rewrite their signing configuration.
set -euo pipefail

source_files() {
 local root=$1 path
 [[ ! -f "$root/sources.list" ]] || printf '%s\n' "$root/sources.list"
 for path in "$root"/sources.list.d/*.list "$root"/sources.list.d/*.sources; do
  [[ ! -f "$path" ]] || printf '%s\n' "$path"
 done
}

matches_source() {
 local path=$1 suite=$2 arch=$3
 awk -v suite="$suite" -v arch="$arch" -v deb822="${path##*.}" '
  function has(value, word, parts, count, i) {
   gsub(/,/, " ", value)
   count=split(value, parts, /[ \t\r\n]+/)
   for (i=1; i<=count; i++) if (parts[i]==word) return 1
   return 0
  }
  function ros_uri(uri) { return uri ~ /^https?:\/\/[^[:space:]]+\/ros2\/ubuntu\/?$/ }
  BEGIN { RS=""; found=0 }
  {
   if (deb822=="sources") {
    for (key in fields) delete fields[key]
    key=""
    lines=split($0, line, "\n")
    for (i=1; i<=lines; i++) {
     if (line[i] ~ /^[ \t]*#/) continue
     if (line[i] ~ /^[^ \t:]+:/) {
      key=line[i]; sub(/:.*/, "", key); key=tolower(key)
      value=line[i]; sub(/^[^:]+:[ \t]*/, "", value)
      fields[key]=value
     } else if (line[i] ~ /^[ \t]/ && key!="") {
      fields[key]=fields[key] " " line[i]
     }
    }
    enabled=tolower(fields["enabled"]); gsub(/[ \t\r]/, "", enabled)
    if (enabled=="no" || enabled=="false" || enabled=="0") next
    if (!has(fields["types"], "deb") || !has(fields["suites"], suite)) next
    if (fields["architectures"]!="" && !has(fields["architectures"], arch)) next
    if (has(fields["architectures-remove"], arch)) next
    count=split(fields["uris"], uris, /[ \t\r\n]+/)
    for (i=1; i<=count; i++) if (ros_uri(uris[i])) {found=1; exit}
   } else {
    lines=split($0, line, "\n")
    for (i=1; i<=lines; i++) {
     entry=line[i]; sub(/#.*/, "", entry); sub(/^[ \t]+/, "", entry)
     if (entry !~ /^deb[ \t]+/) continue
     sub(/^deb[ \t]+/, "", entry); architecture=""; excluded=""
     if (entry ~ /^\[/) {
      options=entry; sub(/\].*/, "", options); sub(/^\[/, "", options)
      count=split(options, option, /[ \t]+/)
      for (j=1; j<=count; j++) {
       if (option[j] ~ /^arch=/) architecture=substr(option[j], 6)
       if (option[j] ~ /^arch-=/) excluded=substr(option[j], 7)
      }
      sub(/^\[[^]]*\][ \t]*/, "", entry)
     }
     split(entry, word, /[ \t]+/)
     if (ros_uri(word[1]) && word[2]==suite &&
         (architecture=="" || has(architecture, arch)) && !has(excluded, arch)) {found=1; exit}
    }
   }
  }
  END {exit !found}
 ' "$path"
}

existing_source() {
 local suite=$1 arch=$2 root=$3 exclude=${4:-} path
 while IFS= read -r path; do
  [[ "$path" != "$exclude" ]] || continue
  if matches_source "$path" "$suite" "$arch"; then
   printf '%s\n' "$path"
   return 0
  fi
 done < <(source_files "$root")
 return 1
}

cleanup_legacy_source() {
 local suite=$1 arch=$2 root=$3 legacy="$3/sources.list.d/ros2.list" expected content existing backup
 [[ -f "$legacy" && ! -L "$legacy" ]] || return 0
 expected="deb [arch=$arch signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $suite main"
 content=$(awk 'NF && $0 !~ /^[ \t]*#/ {print}' "$legacy")
 # Limit automatic repair to the exact single-entry format written by our
 # previous installer. Preserve all user-defined or package-managed sources.
 [[ "$content" == "$expected" ]] || return 0
 if existing=$(existing_source "$suite" "$arch" "$root" "$legacy"); then
  mkdir -p "$root/pm01-source-backups"
  backup=$(mktemp "$root/pm01-source-backups/ros2.list.XXXXXX")
  mv -- "$legacy" "$backup"
  printf 'Backed up duplicate ROS 2 source: %s; reusing %s\n' "$backup" "$existing"
 fi
}

case "${1:-}" in
 exists) existing_source "$2" "$3" "${4:-/etc/apt}" ;;
 cleanup) cleanup_legacy_source "$2" "$3" "${4:-/etc/apt}" ;;
 *) echo 'Usage: ros_apt_source.sh {exists|cleanup} CODENAME ARCH [APT_DIRECTORY]' >&2; exit 2 ;;
esac
