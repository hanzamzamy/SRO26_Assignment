#set text(font: "New Computer Modern", size: 10pt, lang: "id")
#set par(justify: true, first-line-indent: (all: true, amount: 1.8em))
#set heading(numbering: "1.1.")
#set math.equation(numbering: "(1)")
#set page(
  paper: "a4",
  margin: (left: 20mm, right: 20mm, top: 35mm, bottom: 20mm),
  header-ascent: 12mm,
  header: context {
    stack(
      dir: ttb,
      grid(
        columns: (1fr, 1fr),
        align(left)[#image("ITS_pojok.pdf", width: 2cm)], // Pastikan file logo ada di folder yang sama
        align(right)[#text(size: 10pt)[#datetime.today().display("[year]-[month]-[day]")]]
      ),
      v(0.4em),
      line(length: 100%, stroke: 0.5pt)
    )
  }
)

// Bagian Judul
#align(center)[
  #v(1em)
  #text(size: 18pt, weight: "bold")[Laporan Eksplorasi dan Pemetaan Ruang B-202 dengan Pioneer P3DX] 
  #v(1em)
]

// Tabel Metadata
#grid(
  columns: (auto, 1fr),
  gutter: 1em,
  [*Penulis*], [: Rayhan Rizqi Zamzamy],
  [*Dosen*], [: Muhammad Qomaruz Zaman, S.T., M.T., Ph.D.],
  [*Nama Kelas*], [: Sistem Robot Otonom],
  [*Link video YouTube*], [: #link("https://youtu.be/DbYzQwSSMjk")],
  [*Link GitHub*], [: #link("https://github.com/hanzamzamy/SRO26_Assignment/tree/b202_explode")]
)

#v(1.5cm)

= Pendahuluan
Laporan ini menyajikan desain komprehensif dari sistem navigasi otonom pada _mobile robot differential drive_ Pioneer P3DX di dalam lingkungan CoppeliaSim. Tujuan utama dari tugas ini adalah melakukan pemetaan lingkungan (_mapping_) menggunakan susunan sensor ultrasonik, merancang jalur eksplorasi berbasis _grid_, dan mengimplementasikan pelacakan jalur tingkat lanjut (_advanced path tracking_). Sistem ini mengintegrasikan operasi teleoperasi manual untuk pengujian sensor dan _state-machine_ otonom _sense-plan-act_ secara _real-time_.

= Pemodelan dan Algoritma Navigasi
Pemodelan dan analisis kinematika robot didasarkan pada literatur standar robotika @lynch2017modern @siciliano2008springer. Sistem navigasi dibagi menjadi tiga subsistem utama: 
- Pemetaan _grid_ okupansi, 
- Perencanaan jalur A\* (_A-Star_), dan 
- Pengendali _pure pursuit_ termodifikasi. 


== Pemetaan Lingkungan (_Occupancy Grid Mapping_)
Pemetaan dilakukan menggunakan 9 buah sensor ultrasonik bagian depan P3DX. Ruang simulasi didiskritisasi menjadi _grid_ 2D beresolusi $0.1 "m/sel"$. Pembaruan probabilitas keberadaan rintangan menggunakan pendekatan _log-odds_ untuk menghindari _underflow_ numerik. 

Transformasi koordinat pantulan sensor ($p_S$) dari kerangka lokal sensor ke kerangka dunia absolut ($p_W$) didefinisikan sebagai,
$ p_W = ""^W T_B dot ""^B T_S dot p_S $ <eq:transform>
#h(-1.8em)Dimana $""^W T_B$ adalah matriks transformasi dari koordinat dunia ke bodi robot, dan $""^B T_S$ adalah transformasi dari bodi ke sensor.

Sinar pantul sensor direpresentasikan ke dalam sel-sel _grid_ menggunakan algoritma garis Bresenham. Pembaruan probabilitas okupansi untuk sel _grid_ $m_(x,y)$ pada iterasi waktu ke-$t$ menggunakan pembaruan Bayesian Log-Odds dirumuskan sebagai,
$ op(l_t) (m_(x,y)) = op(l_(t-1)) (m_(x,y)) + op(l_"inv")(m_(x,y) | z_t, x_t) - l_0 $ <eq:logodds>

#h(-1.8em)Di mana $l_"inv"$ adalah representasi _inverse sensor model_ dalam ruang _log-odds_, $z_t$ adalah pengukuran sensor, dan $x_t$ adalah pose bodi robot pada waktu $t$. Nilai inversi ini didefinisikan secara komprehensif sebagai,
$ l_"inv"(m_(x,y) | z_t, x_t) = log ( P(m_(x,y) | z_t, x_t) / (1 - P(m_(x,y) | z_t, x_t)) ) $ <eq:inv_sensor>

Dalam implementasi praktisnya, nilai $l_"inv"$ ditetapkan sebesar $l_"occ" = 0.85$ jika sel merupakan titik akhir pantulan (rintangan), dan $l_"free" = -0.5$ untuk setiap sel yang hanya dilalui oleh sinar (ruang bebas). Probabilitas okupansi awal (_prior_) diasumsikan seimbang, sehingga $l_0 = 0$. Untuk menyaring _noise_ (_ghost obstacles_), sebuah sel hanya dideklarasikan sebagai rintangan absolut apabila total _log-odds_ melampaui $1.5$ (mensyaratkan minimal 2x pantulan konsekutif).

== Perencanaan Jalur Jarak Terpendek (A\* dengan _Gradient Costmap_)
Peta tidak diperlakukan sebagai halangan biner, tetapi diekspansi (_inflation_) membentuk _gradient costmap_ secara sirkular. Radius fisik robot ($r_"lethal" = 0.3 "m"$) diberi nilai penalti $100.0$, merepresentasikan tidak dapat dilewati. Area penyangga keamanan ($r_"risk" = 0.5 "m"$) diberikan penalti yang menurun secara linear.
$ C_"risk" = 40.0 times (1.0 - (d - r_"lethal") / (r_"risk" - r_"lethal")) $ <eq:penalty>

Algoritma A\* mencari jalur optimal dengan meminimalkan fungsi biaya total $f(n)$:
$ f(n) = g(n) + h(n) + (C_"risk" times 2.0) $ <eq:astar>
#h(-1.8em)Hal ini memaksa algoritma A\* untuk merencanakan jalur di tengah ruangan terbuka, namun tetap mengizinkan robot masuk di lorong sempit dengan mengorbankan penalti $C_"risk"$ jika tidak ada alternatif lain.

== Pelacakan Jalur (_Pure Pursuit_ dengan _Point-Turn_)
Algoritma _pure pursuit_ standar tidak cocok untuk robot _differential drive_ saat menghadapi tikungan patah karena algoritma tersebut didesain untuk kemudi Ackermann. Oleh karena itu, pengontrol dimodifikasi dengan penambahan batas toleransi putar-di-tempat (_point-turn_).

Koordinat target _look-ahead_ ($L_d$) ditransformasikan ke kerangka lokal bodi robot ($x_r, y_r$). Kelengkungan lintasan ($gamma$) dihitung menggunakan persamaan geometris,
$ gamma = (2 y_r) / (L_d^2) $ <eq:curvature>

#h(-1.8em)Kecepatan sudut rotasi robot ($omega$) dikalkulasi secara proporsional terhadap kelengkungan $gamma$. Jika target berada pada sudut tajam di atas 0.6 radian, robot akan mengubah kecepatan translasi $V$ menjadi $0$ dan berputar di tempat.
$ omega = 1.5 times arctan(y_r / x_r) $ <eq:pointturn>

#h(-1.8em)Berdasarkan kinematika _differential drive_ @lynch2017modern, kecepatan masing-masing roda aktuator ($v_r, v_l$) dengan radius roda $R_w$ dan separuh jarak antar roda $R_b$ adalah,
$ v_r = (V + R_b omega) / R_w, quad v_l = (V - R_b omega) / R_w $ <eq:kinematics>

= Implementasi dan Pengujian Sistem
Subsistem pemetaan _grid_ okupansi yang telah dibuat diuji menggunakan skema kendali _remote_. Robot bergerak berdasarkan _command input_ dari _keyboard_. Pengujian subsistem _mapping_ sangant krusial karena subsistem lainnya, seperti perancangan jalur A\* dan pengendali _pure pursuit_, sangat bergantung pada akurasi peta yang dihasilkan. Poin-poin penting dalam pengujian ini di antaranya,
- Transformasi koordinat sensor yang benar,
- Pembaruan probabilitas okupansi yang konsisten, dan 
- Kemampuan untuk menyaring _noise_ secara efektif. 

#h(-1.8em)Setelah validasi pemetaan, sistem beralih ke mode otonom dengan _state-machine_.

#figure(
  image("final_map_bak.png", width: 50%),
  caption: [Proses pemetaan manual. Garis hitam merupakan dinding (probabilitas tinggi), area putih adalah ruang bebas.],
) <fig:map_manual>

Implementasi akhir menggunakan arsitektur _state-machine_ (`WAITING`, `PLANNING`, `TRACKING`). Antarmuka pengguna interaktif dibangun menggunakan `matplotlib` dimana pengguna dapat menentukan titik tujuan eksplorasi baru di area yang belum terpetakan.

#figure(
  image("live_map.png", width: 50%),
  caption: [Mode otonom. Titik biru adalah posisi robot, garis merah adalah jalur A\* yang sedang diikuti, titik hijau adalah tujuan eksplorasi, dan area abu-abu menunjukkan area yang belum terpetakan.],
) <fig:map_auto>

#h(-1.8em)Selain itu, sistem dilengkapi dengan _software bumper_. Jika robot mendeteksi perubahan posisinya kurang dari $0.03 "m"$ dalam $1.5$ detik saat motor diaktifkan, yang indikasi terjepit di dinding, robot akan mundur sejenak dan membatalkan jalur, memaksa A\* melakukan _replanning_.

Peta okupansi dievaluasi oleh A\* apabila ada hambatan dinamis yang muncul secara tiba-tiba di jalur yang sedang diikuti, misalnya ketika melalui area yang belum terpetakan. 

#figure(
  grid(
    columns: (1fr, 1fr),
    align(right)[#text(size: 9pt)[
      #image("qabla_jalan_singkat.png", width: 50%)
      ]], 
    align(left)[#text(size: 9pt)[
      #image("bada_jalan_singkat.png", width: 50%)
      ]]
  ),
  caption: [Kiri: Jalur A\* awal (garis merah) yang direncanakan sebelum robot menemui rintangan tak terpetakan. Kanan: Jalur A\* yang diperbarui secara seketika setelah robot mendeteksi rintangan, menunjukkan kemampuan _live replanning_. Pada kasus ini, jalur terpendek yang baru menghindari rintangan.],
) <fig:shortest_path>

#h(-1.8em)Inflasi peta okupansi yang membentuk _gradient costmap_ terbukti efektif dalam memandu A\* untuk merencanakan jalur yang lebih aman, meskipun terkadang mengorbankan jarak tempuh terpendek karena mengasumsikan area yang akan dilalui cukup sempit untuk dilalui. 

#figure(
  grid(
    columns: (1fr, 1fr),
    align(right)[#text(size: 9pt)[
      #image("qabla_jalan_sempit.png", width: 50%)
      ]], 
    align(left)[#text(size: 9pt)[
      #image("bada_jalan_sempit.png", width: 50%)
      ]]
  ),
  caption: [Kiri: Jalur A\* awal (garis merah) yang direncanakan sebelum robot menemui rintangan celah sempit. Kanan: Jalur A\* yang diperbarui secara seketika setelah robot mendeteksi rintangan. Pada kasus ini, robot memilih memutar walaupun jalur terpendek tetap bisa dilalui dengan margin yang sangat kecil.],
) <fig:narrow_path>

Algoritma A\* yang diimplementasikan juga terbukti mengenali _grid_ yang tidak dapat dituju (misal: area di luar ruangan) setelah menjelajahi peta tertutup secara menyeluruh, sehingga tidak terjebak dalam loop perencanaan yang tidak berujung. Area di dalam ruangan masih dapat dijelajahi walaupun belum pernah dilalui sebelumnya, karena robot dapat melakukan _replanning_ secara dinamis saat menemui rintangan tak terpetakan.

#figure(
  grid(
    columns: (1fr, 1fr),
    align(right)[#text(size: 9pt)[
      #image("final_map.png", width: 50%)
      ]], 
    align(left)[#text(size: 9pt)[
      #image("final_map_inside.png", width: 50%)
      ]]
  ),
  caption: [Kiri: Robot berhenti mengejar tujuan di luar ruangan setelah memetakan seluruh area tertutup. Kanan: Robot masih dapat menjelajahi area di dalam ruangan yang belum pernah dilalui sebelumnya.],
) <fig:map_final>

= Kesimpulan
Sistem navigasi otonom yang dikembangkan telah memenuhi objektif desain eksplorasi robot. Implementasi _occupancy grid mapping_ berbasis _log-odds_ mampu menghasilkan peta biner yang tangguh terhadap _noise_. Kombinasi dari _gradient costmap_ pada A\* dan modifikasi _point-turn_ pada _pure pursuit_ terbukti sukses menyelesaikan dilema lorong sempit (_narrow passage problem_), sekaligus mengeliminasi masalah tabrakan sudut (_corner-clipping_) pada batasan sistem non-holonomik.

#bibliography("references.bib", style: "ieee")