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
        align(left)[#image("ITS_pojok.pdf", width: 2cm)],
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
  #text(size: 18pt)[Permainan Sepak Bola Menggunakan Pioneer P3DX] 
  #v(1em)
]

// Tabel Metadata
#grid(
  columns: (auto, 1fr),
  gutter: 1em,
  [*Penulis*], [: Rayhan Rizqi Zamzamy],
  [*Dosen*], [: Muhammad Qomaruz Zaman, S.T., M.T., Ph.D.],
  [*Nama Kelas*], [: Sistem Robot Otonom],
  [*Link video YouTube*], [: #link("https://youtu.be/j83UhDDxh8s")],
  [*Link GitHub*], [: #link("https://github.com/hanzamzamy/SRO26_Assignment/tree/midtest")]
)
#v(1.5cm)

= Pendahuluan
Laporan ini membahas perancangan dan implementasi sistem multi-robot otonom dalam simulasi CoppeliaSim. Tujuan dari simulasi ini adalah untuk mendemonstrasikan koordinasi tiga unit robot _differential drive_ Pioneer P3DX yang diberikan peran spesifik (_Striker_, _Goalkeeper_, dan _Passer_) dalam skenario permainan sepak bola sederhana. Sistem dikendalikan secara tersentralisasi melalui ZeroMQ Remote API menggunakan bahasa pemrograman Python. Robot dirancang untuk memanipulasi objek (bola) secara murni melalui transfer momentum mekanis (tumbukan fisik) dengan mengandalkan kendali kinematika dan _Finite State Machine_.

= Desain Sistem dan Pembagian Peran
Sistem terdiri dari tiga entitas otonom yang beroperasi pada lingkungan yang sama. Koordinasi antar-robot dijembatani oleh kelas `GameState` yang melacak kemajuan fase permainan secara global, seperti status gol dan kesiapan penerima umpan.

== Penyerang Utama (_Striker Robot_)
Robot ini bertugas sebagai unit ofensif utama. Algoritma robot ini mencakup kalkulasi trajektori penembakan cerdas (_smart aim_). Sebelum mengeksekusi tembakan, robot mengekstraksi posisi _Goalkeeper_ secara _real-time_ dan menghitung titik target optimal pada sisi gawang (lebar 2.2 m) yang tidak terjaga. Siklus operasinya meliputi tembakan bola pertama (merah), bermanuver ke titik siaga untuk menerima umpan, menjebak (_intercept_) bola kedua (biru), dan mengeksekusi tembakan akhir.

== Penjaga Gawang (_Goalkeeper Robot_)
Robot ini bertindak sebagai unit defensif. Pada kondisi standar, robot melakukan patroli osilasi sinusoidal di sepanjang garis gawang. Jika tembakan bola pertama (merah) berhasil mencetak gol, kondisi robot berubah menjadi pelacak bola dan tidak melakukan osilasi secara buta. Ketika bola memasuki radius pertahanan (< 3 meter), status FSM robot berubah menjadi pelacakan aktif (_chase ball_) untuk memotong lintasan bola secara langsung @corke2011robotics. Robot ini hanya akan berpindah fokus dari bola merah ke bola biru setelah `GameState` memverifikasi bahwa gol pertama telah sah secara spasial.

== Pengumpan (_Passer Robot_)
Berperan sebagai unit pendukung. Algoritma robot ini dilengkapi dengan manuver navigasi aman (_navigate around ball_) untuk menghindari benturan prematur dengan bola saat melakukan _positioning_. Robot ini akan bersiaga tepat di belakang bola biru dan baru akan melakukan eksekusi umpan (menumbuk bola menuju _Striker_) setelah mendeteksi bahwa _Striker_ telah berada pada posisi menerima umpan dan gol pertama telah tercetak.

#figure(
  image("initial_state.png", width: 85%),
  caption: [Kondisi awal _scene_ sepak bola pada CoppeliaSim.],
)

= Desain Pengendali dan Navigasi

== Kendali Proporsional Diferensial
Seluruh robot mewarisi metode navigasi dari kelas dasar `RobotP3DX` dan `RoleBehaviorMixin`. Untuk mencapai titik target $(x_t, y_t)$, robot menghitung eror sudut, sudut _heading_ referensi dikurangi sudut aktual. Keluaran kecepatan roda kiri ($v_l$) dan kanan ($v_r$) dihitung menggunakan metode kendali proporsional @tzafestas2013introduction:

$ v_l = V_"base" - (K_p times theta_"err") $
$ v_r = V_"base" + (K_p times theta_"err") $

Nilai kecepatan dasar ($V_"base"$) diturunkan secara dinamis berdasarkan besaran eror sudut. Jika eror sudut terlampau tajam ($> \pi/4$), kecepatan linear dasar direduksi menjadi nol sehingga robot dapat berputar di tempat (_point turn_) secara stabil tanpa kehilangan orientasi target.

== Mekanisme Tumbukan (_Physical Ramming_)
Untuk memindahkan bola, robot menggunakan pendekatan tumbukan fisik murni. Ketika robot berada pada jarak dekat dengan bola ($< 0.7$ meter) dan orientasi telah sejajar dengan target akhir, algoritma memicu status dorongan (_dash_). Dalam waktu sesaat ($0.5$ detik), kecepatan roda dibatasi dan dimaksimalkan pada nilai batas absolut ($V_"max" = 15.0 " rad/s"$). Manuver agresif ini mentransfer momentum kinetik dari sasis robot ke massa bola, menciptakan efek tendangan fisik yang realistis. Setelah limit waktu _dash_ tercapai, robot melakukan pengereman otomatis ($v_l = 0, v_r = 0$) agar tidak menggiring bola tanpa henti.

= Hasil Simulasi dan Evaluasi
Simulasi dijalankan dalam mode _synchronous stepping_ untuk menjamin stabilitas integrasi fisika mesin simulasi. Berdasarkan hasil iterasi eksperimental:
+ _Striker_ berhasil memprediksi celah gawang dan mengeksekusi tembakan bola merah tanpa mengenai _Goalkeeper_.
+ Sinkronisasi _Game State_ berfungsi optimal. _Passer_ menahan posisi statisnya, dan hanya melepaskan umpan bola biru sesaat setelah fungsi kalkulasi jarak mendeteksi bahwa bola merah telah sepenuhnya melewati garis gawang ($y = 0.0, x > 1.0$).
+ Penambahan status _Intercept_ pada _Striker_ terbukti krusial. _Striker_ menerapkan pengereman darurat untuk menjebak bola secara fisik, memastikan trajektori tembakan kedua tetap presisi.

#figure(
  image("sim1.png", width: 85%),
  caption: [Setiap robot melakukan _positioning_ dan manuver sesuai peran masing-masing.],
)

#figure(
  image("sim2.png", width: 85%),
  caption: [_Striker_ menembak bola merah, _Passer_ menunggu di posisi, dan _Goalkeeper_ melakukan patroli osilasi.],
)

#figure(
  image("sim4.png", width: 85%),
  caption: [Gol pertama tercetak, _Passer_ mengeksekusi umpan, dan _Striker_ menangkap bola biru.],
)

#figure(
  image("sim5.png", width: 85%),
  caption: [_Striker_ memposisikan diri untuk tembakan akhir.],
)

#figure(
  image("sim6.png", width: 85%),
  caption: [_Striker_ menembak bola biru ke gawang, _Goalkeeper_ memposisikan diri untuk mengantisipasi.],
)

#figure(
  image("sim7.png", width: 85%),
  caption: [_Goalkeeper_ mencegah gol kedua.],
)

#figure(
  image("sim8.png", width: 85%),
  caption: [_Goalkeeper_ menggiring bola biru menjauh.],
)

= Kesimpulan
Implementasi sistem multi-robot ini membuktikan bahwa koordinasi taktis kompleks dapat dicapai dengan mengkombinasikan _Finite State Machine_ lokal pada masing-masing entitas dengan satu variabel sinkronisasi global. Penggunaan metode kendali proporsional dinamis memberikan stabilitas navigasi yang baik. Selain itu, eksekusi manipulasi objek dengan memanfaatkan tumbukan mekanis langsung (berbasis batas waktu _burst velocity_) menawarkan solusi interaksi fisik yang jauh lebih stabil dan realistis dibandingkan dengan penerapan gaya buatan melalui API secara langsung.

#bibliography("references.bib", style: "ieee")