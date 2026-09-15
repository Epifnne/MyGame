param(
    [Parameter(Mandatory = $false)]
    [string]$Key = "list"
)

$ue = "E:\unreal src\UnrealEngine-release"

$map = @{
    "onrep" = @{ Path = "Engine\Source\Runtime\Engine\Private\ActorReplication.cpp"; Line = 179; Desc = "AActor::OnRep_ReplicatedMovement 复制分流入口" }
    "postphys" = @{ Path = "Engine\Source\Runtime\Engine\Private\ActorReplication.cpp"; Line = 298; Desc = "AActor::PostNetReceivePhysicState 刚体目标下发" }
    "actor_repmove" = @{ Path = "Engine\Source\Runtime\Engine\Classes\GameFramework\Actor.h"; Line = 794; Desc = "ReplicatedMovement 字段" }
    "actor_replicate_flag" = @{ Path = "Engine\Source\Runtime\Engine\Classes\GameFramework\Actor.h"; Line = 619; Desc = "bReplicateMovement 开关位" }
    "rbstate" = @{ Path = "Engine\Source\Runtime\Engine\Classes\Engine\ReplicatedState.h"; Line = 85; Desc = "FRigidBodyState 结构" }
    "repmove" = @{ Path = "Engine\Source\Runtime\Engine\Classes\Engine\ReplicatedState.h"; Line = 118; Desc = "FRepMovement 结构" }
    "phy_cpp" = @{ Path = "Engine\Source\Runtime\Engine\Private\PhysicsEngine\PhysicsReplication.cpp"; Line = 1; Desc = "PhysicsReplication 主要实现" }
    "phy_h" = @{ Path = "Engine\Source\Runtime\Engine\Public\PhysicsReplication.h"; Line = 1; Desc = "FPhysicsReplicationAsync 声明" }
    "netphy_h" = @{ Path = "Engine\Source\Runtime\Engine\Public\Physics\NetworkPhysicsComponent.h"; Line = 1321; Desc = "UNetworkPhysicsComponent" }
    "netphy_tick" = @{ Path = "Engine\Source\Runtime\Engine\Private\PhysicsEngine\NetworkPhysicsComponent.cpp"; Line = 667; Desc = "UNetworkPhysicsComponent::TickComponent" }
}

if ($Key -eq "list") {
    Write-Host "Available keys:" -ForegroundColor Cyan
    $map.Keys | Sort-Object | ForEach-Object {
        $entry = $map[$_]
        Write-Host ("  {0} -> {1}:{2} ({3})" -f $_, $entry.Path, $entry.Line, $entry.Desc)
    }
    exit 0
}

if (-not $map.ContainsKey($Key)) {
    Write-Error "Unknown key: $Key. Run: .\\Tools\\UE_NetPhysics_Jump.ps1 list"
    exit 1
}

$entry = $map[$Key]
$target = Join-Path $ue $entry.Path

if (-not (Test-Path $target)) {
    Write-Error "File not found: $target"
    exit 1
}

$codeCmd = Get-Command code -ErrorAction SilentlyContinue
if (-not $codeCmd) {
    Write-Error "VS Code CLI 'code' not found in PATH. Install shell command from VS Code and retry."
    exit 1
}

Write-Host ("Opening {0}:{1} - {2}" -f $target, $entry.Line, $entry.Desc) -ForegroundColor Green
& code -g "$target`:$($entry.Line)"


