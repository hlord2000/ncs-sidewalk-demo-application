# Source this to get a working west and NCS toolchain for this workspace.
#
#   source ncs-sidewalk-demo-application/tools/ncs-env.sh
#
# The toolchain bundle ships its own Python, which needs LD_LIBRARY_PATH
# pointing into the bundle or west fails to start. Set NCS_TOOLCHAIN to pick a
# specific bundle; otherwise the newest one under NCS_TOOLCHAIN_ROOT is used.

# Resolve this script's directory whether sourced from bash or zsh.
if [ -n "${BASH_SOURCE[0]:-}" ]; then
	_ncs_env_self="${BASH_SOURCE[0]}"
elif [ -n "${ZSH_VERSION:-}" ]; then
	_ncs_env_self="${(%):-%x}"
else
	echo "ncs-env.sh: source this from bash or zsh" >&2
	return 1 2>/dev/null || exit 1
fi

_ncs_env_tools="$(cd "$(dirname "$_ncs_env_self")" && pwd)"
# tools/ -> repo root -> west workspace root
export ZEPHYR_BASE="$(cd "$_ncs_env_tools/../.." && pwd)/zephyr"

# Toolchain bundles live in two places depending on how NCS was installed:
#   ~/ncs/toolchains/<hash>/                          (nRF Connect for Desktop)
#   /opt/ncs/toolchains/ncs-vX.Y.Z/toolchains/<hash>/ (system-wide install)
# Set NCS_TOOLCHAIN to pick one explicitly. Otherwise the newest across both
# roots wins. Match the Zephyr SDK to the NCS version this manifest pins:
# NCS v3.0.0 wants Zephyr SDK 0.17.0, which the ncs-v3.2.1 bundle also ships.
if [ -z "${NCS_TOOLCHAIN:-}" ]; then
	for _ncs_root in ${NCS_TOOLCHAIN_ROOT:-} "$HOME/ncs/toolchains" /opt/ncs/toolchains/*/toolchains; do
		[ -d "$_ncs_root" ] || continue
		for _cand in $(ls -1dt "$_ncs_root"/*/ 2>/dev/null); do
			if [ -x "${_cand%/}/usr/local/bin/west" ]; then
				NCS_TOOLCHAIN="${_cand%/}"
				break
			fi
		done
		[ -n "${NCS_TOOLCHAIN:-}" ] && break
	done
	unset _ncs_root _cand
fi
NCS_TOOLCHAIN="${NCS_TOOLCHAIN%/}"

if [ ! -d "$NCS_TOOLCHAIN" ]; then
	echo "ncs-env.sh: no toolchain found under $NCS_TOOLCHAIN_ROOT." >&2
	echo "ncs-env.sh: install one with nRF Connect for Desktop, or set NCS_TOOLCHAIN." >&2
	return 1 2>/dev/null || exit 1
fi

export PATH="$NCS_TOOLCHAIN/usr/local/bin:$NCS_TOOLCHAIN/bin:$PATH"
export LD_LIBRARY_PATH="$NCS_TOOLCHAIN/usr/local/lib:$NCS_TOOLCHAIN/lib/x86_64-linux-gnu:$NCS_TOOLCHAIN/usr/lib/x86_64-linux-gnu:$NCS_TOOLCHAIN/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_INSTALL_DIR="$NCS_TOOLCHAIN/opt/zephyr-sdk"

unset _ncs_env_self _ncs_env_tools
